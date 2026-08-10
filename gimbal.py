"""Pan/tilt control for the scanner head.

Two backends, same interface:

* "hwpwm"    - writes period and duty cycle in nanoseconds straight to /sys/class/pwm.
               The kernel's PWM peripheral generates the pulses, so what you ask for is
               what appears on the pin. Requires dtoverlay=pwm-2chan and the servos on
               GPIO12/GPIO13. This is the backend to trust.
* "gpiozero" - the original AngularServo path. Software-timed pulses whose real width
               depends on the pin factory honouring the requested frame period. Kept for
               comparison, not recommended: on this rig a commanded 0-180 sweep produced
               roughly 90 degrees of travel, which means the delivered pulse widths did
               not span the range that was asked for.

The MG90S has no position feedback, so `pan` and `tilt` report the last commanded angle.
The IMU rides on this head and is the only independent check available; see verify_tilt().
"""

import argparse
import os
import time

# --- Mechanical calibration -------------------------------------------------------------
# Measured on the assembled gimbal.

PAN_PIN = 12  # hardware PWM channel 0
TILT_PIN = 13  # hardware PWM channel 1

PAN_CHANNEL = 0
TILT_CHANNEL = 1

# Legacy pins, only meaningful for the gpiozero backend.
PAN_PIN_GPIOZERO = 17
TILT_PIN_GPIOZERO = 27

# Angle scale and pulse endpoints, per axis. Measured against the assembled gimbal with
# --pulse-sweep, so these describe this rig rather than the servo in the abstract. Each axis
# is anchored on two observed points and interpolated linearly between them.
#
# Pan: 400us at 0 degrees, 2400us at 180 degrees.
# Tilt: 400us straight up, 1350us at true horizontal. Angles are expressed as 0 = straight
#       up, 90 = horizontal.
#
# The two axes do not share a scale -- 11.11us/deg on pan against 10.56us/deg on tilt. For
# two identical servos driven directly that is a little surprising, and it is worth
# re-checking the tilt endpoints if elevation ever looks systematically off. Unit variation
# and any non-1:1 linkage on the tilt axis both land here. The empirical numbers win either
# way; nothing downstream assumes the scales match.
#
# Note the pulse range was never the original problem. Under the gpiozero/lgpio software-PWM
# backend a commanded 0-180 sweep produced about 90 degrees of travel and skipped steps,
# because the delivered pulse widths did not match the requested ones. Driving the kernel PWM
# peripheral directly fixed it outright. Suspect the pulse source before suspecting the
# servo, and treat any calibration predating the hwpwm switch as void.
PAN_SERVO_RANGE = (0, 180)
PAN_PULSE_US = (400, 2400)

TILT_SERVO_RANGE = (0, 90)  # 0 straight up, 90 true horizontal
TILT_PULSE_US = (400, 1350)

# Hard travel bounds, per axis, in pulse width. Every pulse is clamped to these before it
# reaches the hardware, so a bad angle limit or an arithmetic slip cannot drive a servo into
# its mechanical stop or aim the beam somewhere useless.
#
# Tilt deliberately reaches below its 400us calibration anchor: 300us tips the head about
# 9.5 degrees past vertical, which is wanted, while the upper bound stops dead at 1350us
# because anything past horizontal only ever measured the rig's own hardware.
#
# The corresponding angle limits are derived from these below rather than written out, so
# re-calibrating the anchors keeps everything consistent.
PAN_PULSE_LIMITS_US = (400, 2400)
TILT_PULSE_LIMITS_US = (300, 1350)

FRAME_US = 20000  # 50Hz, the standard servo frame

# Datasheet operating speed is 0.1s/60deg (600 deg/s) unloaded at 4.8V. Under the mass of
# the LiDAR and IMU it will be slower, so budget conservatively.
SLEW_RATE = 300.0  # deg/s
SETTLE_TIME = 0.15  # seconds of ring-down after arrival

# Smallest step this rig delivers reliably, established with --step-test on hardware PWM.
# 1 degree works; 2 is the safer choice for a long scan, since gear backlash rather than
# pulse resolution is the limit now.
MIN_RELIABLE_STEP = 1.0  # degrees

PWM_SYSFS_ROOT = "/sys/class/pwm"


def _clamp(value, low, high):
    return max(low, min(high, value))


def _angle_to_pulse_us(angle, angle_range, pulse_range):
    """Linear map from an axis' angle scale onto its own pulse-width range.

    Extrapolates outside `angle_range` on purpose: tilt is calibrated on 0-90 degrees but
    travels past horizontal, so angles above 90 map to pulse widths above 1350us.
    """
    low, high = angle_range
    min_pulse, max_pulse = pulse_range
    fraction = (angle - low) / (high - low)
    return min_pulse + fraction * (max_pulse - min_pulse)


def _pulse_us_to_angle(pulse_us, angle_range, pulse_range):
    """Inverse of _angle_to_pulse_us."""
    low, high = angle_range
    min_pulse, max_pulse = pulse_range
    fraction = (pulse_us - min_pulse) / (max_pulse - min_pulse)
    return low + fraction * (high - low)


def _angle_limits(pulse_limits, angle_range, pulse_range):
    """Converts an axis' hard pulse bounds into the angle range they correspond to."""
    return tuple(
        round(_pulse_us_to_angle(pulse, angle_range, pulse_range), 1)
        for pulse in pulse_limits
    )


# Safe travel limits, in each axis' own angle scale, derived from the pulse bounds above.
# Commands outside these are clamped rather than raising, so a scan pattern that overshoots
# cannot drive the head into its own mounting.
PAN_LIMITS = _angle_limits(PAN_PULSE_LIMITS_US, PAN_SERVO_RANGE, PAN_PULSE_US)
TILT_LIMITS = _angle_limits(TILT_PULSE_LIMITS_US, TILT_SERVO_RANGE, TILT_PULSE_US)


class HardwarePWMAxis:
    """One servo driven by the kernel PWM peripheral through sysfs.

    Everything is expressed in nanoseconds because that is what the kernel interface takes.
    No scaling, no duty-cycle percentages, no assumptions about frame period held somewhere
    else in the stack.
    """

    def __init__(self, channel, angle_range, pulse_range, pulse_limits, chip=None):
        self.angle_range = angle_range
        self.pulse_range = pulse_range
        self.pulse_limits = pulse_limits
        self.chip_path = _find_pwmchip(chip)
        self.channel = channel
        self.path = os.path.join(self.chip_path, f"pwm{channel}")

        if not os.path.isdir(self.path):
            _write(os.path.join(self.chip_path, "export"), str(channel))
            # The kernel creates the directory asynchronously, and when a udev rule is
            # fixing up ownership there is a further gap before the files are writable.
            period_path = os.path.join(self.path, "period")
            for _ in range(100):
                if os.path.isdir(self.path) and os.access(period_path, os.W_OK):
                    break
                time.sleep(0.01)
            else:
                raise RuntimeError(
                    f"PWM channel {channel} never became writable at {self.path}"
                )

        # Order matters: period must be set before a duty cycle that would exceed the old
        # period, and duty must be non-zero before enabling or the servo sees no pulse.
        _write(os.path.join(self.path, "period"), str(FRAME_US * 1000))
        self._pulse_ns = int(_clamp(sum(pulse_range) / 2, *pulse_limits) * 1000)
        _write(os.path.join(self.path, "duty_cycle"), str(self._pulse_ns))
        self.enabled = False

    def set_pulse_us(self, pulse_us):
        """Drives a raw pulse width. The calibration primitive."""
        pulse_us = _clamp(pulse_us, *self.pulse_limits)
        self._pulse_ns = int(pulse_us * 1000)
        _write(os.path.join(self.path, "duty_cycle"), str(self._pulse_ns))
        if not self.enabled:
            _write(os.path.join(self.path, "enable"), "1")
            self.enabled = True

    def set_angle(self, angle):
        self.set_pulse_us(_angle_to_pulse_us(angle, self.angle_range, self.pulse_range))

    def detach(self):
        """Stops the pulse train. The servo holds position by friction, if at all."""
        if self.enabled:
            _write(os.path.join(self.path, "enable"), "0")
            self.enabled = False

    def close(self):
        self.detach()


class GpiozeroAxis:
    """One servo driven by gpiozero's AngularServo. Software-timed; kept for comparison."""

    def __init__(self, pin, angle_range, pulse_range, pulse_limits):
        from gpiozero import AngularServo

        self.angle_range = angle_range
        self.pulse_range = pulse_range
        self.pulse_limits = pulse_limits
        self.servo = AngularServo(
            pin, initial_angle=None,
            min_angle=angle_range[0], max_angle=angle_range[1],
            min_pulse_width=pulse_range[0] / 1_000_000,
            max_pulse_width=pulse_range[1] / 1_000_000,
        )

    def set_pulse_us(self, pulse_us):
        raise NotImplementedError(
            "Raw pulse control needs the hwpwm backend; gpiozero only exposes angles."
        )

    def set_angle(self, angle):
        self.servo.angle = angle

    def detach(self):
        self.servo.angle = None

    def close(self):
        self.detach()
        self.servo.close()


def _find_pwmchip(preferred=None):
    """Locates the PWM chip exposed by dtoverlay=pwm-2chan.

    The number is not stable across kernels and models, so scan rather than hardcode.
    """
    if preferred is not None:
        path = os.path.join(PWM_SYSFS_ROOT, f"pwmchip{preferred}")
        if os.path.isdir(path):
            return path
        raise RuntimeError(f"{path} does not exist")

    if not os.path.isdir(PWM_SYSFS_ROOT):
        raise RuntimeError(
            f"{PWM_SYSFS_ROOT} does not exist. Add 'dtoverlay=pwm-2chan,pin=12,func=4,"
            f"pin2=13,func2=4' to /boot/firmware/config.txt and reboot."
        )

    chips = sorted(name for name in os.listdir(PWM_SYSFS_ROOT) if name.startswith("pwmchip"))
    if not chips:
        raise RuntimeError(
            f"No pwmchip under {PWM_SYSFS_ROOT}. Is dtoverlay=pwm-2chan in "
            f"/boot/firmware/config.txt, and did you reboot?"
        )
    return os.path.join(PWM_SYSFS_ROOT, chips[0])


def _write(path, value):
    try:
        with open(path, "w") as handle:
            handle.write(value)
    except PermissionError as exc:
        raise PermissionError(
            f"Cannot write {path}. The PWM sysfs interface is root-owned by default; "
            f"run with sudo, or add a udev rule granting your user access."
        ) from exc


class Gimbal:
    """Commands the pan/tilt head and tracks where it was told to go."""

    def __init__(self, backend="hwpwm", slew_rate=SLEW_RATE, settle_time=SETTLE_TIME,
                 hold=True):
        self.slew_rate = slew_rate
        self.settle_time = settle_time
        self.hold = hold
        self.backend = backend

        if backend == "hwpwm":
            self.pan_axis = HardwarePWMAxis(
                PAN_CHANNEL, PAN_SERVO_RANGE, PAN_PULSE_US, PAN_PULSE_LIMITS_US)
            self.tilt_axis = HardwarePWMAxis(
                TILT_CHANNEL, TILT_SERVO_RANGE, TILT_PULSE_US, TILT_PULSE_LIMITS_US)
        elif backend == "gpiozero":
            self.pan_axis = GpiozeroAxis(
                PAN_PIN_GPIOZERO, PAN_SERVO_RANGE, PAN_PULSE_US, PAN_PULSE_LIMITS_US)
            self.tilt_axis = GpiozeroAxis(
                TILT_PIN_GPIOZERO, TILT_SERVO_RANGE, TILT_PULSE_US, TILT_PULSE_LIMITS_US)
        else:
            raise ValueError(f"unknown backend {backend!r}")

        # None means "position unknown" -- true at startup and after release().
        self._pan = None
        self._tilt = None

    @property
    def pan(self):
        """Last commanded pan angle, or None if the position is unknown."""
        return self._pan

    @property
    def tilt(self):
        """Last commanded tilt angle, or None if the position is unknown."""
        return self._tilt

    def _travel_time(self, current, target, span):
        distance = span if current is None else abs(target - current)
        return distance / self.slew_rate

    def move_to(self, pan=None, tilt=None, settle=True):
        """Moves one or both axes and blocks until the head should have arrived.

        Axes are commanded together and waited on once, so a diagonal move costs its longer
        component rather than the sum. Returns the (pan, tilt) actually commanded.
        """
        travel = 0.0

        if pan is not None:
            pan = _clamp(pan, *PAN_LIMITS)
            travel = max(travel, self._travel_time(
                self._pan, pan, PAN_LIMITS[1] - PAN_LIMITS[0]))
        if tilt is not None:
            tilt = _clamp(tilt, *TILT_LIMITS)
            travel = max(travel, self._travel_time(
                self._tilt, tilt, TILT_LIMITS[1] - TILT_LIMITS[0]))

        if pan is not None:
            self.pan_axis.set_angle(pan)
            self._pan = pan
        if tilt is not None:
            self.tilt_axis.set_angle(tilt)
            self._tilt = tilt

        if settle:
            time.sleep(travel + self.settle_time)
            if not self.hold:
                # Cutting the pulse train stops the servo hunting, which matters because the
                # IMU shares this platform and reads the buzz as real acceleration. The cost
                # is that a detached servo applies no holding torque at all.
                self.release()

        return self._pan, self._tilt

    def release(self):
        """Stops driving both servos."""
        self.pan_axis.detach()
        self.tilt_axis.detach()

    def home(self):
        return self.move_to(pan=PAN_LIMITS[0], tilt=TILT_LIMITS[1])

    def scan_positions(self, pan_start, pan_end, pan_step, tilt_start, tilt_end, tilt_step):
        """Yields (pan, tilt) at every grid point, moving and settling before each yield.

        A generator so the caller owns the measurement: take the LiDAR range and IMU
        attitude at each yield, then resume. Serpentine ordering reverses alternate rows so
        the head never travels the full pan span between them.
        """
        for step, name in ((pan_step, "pan"), (tilt_step, "tilt")):
            if abs(step) < MIN_RELIABLE_STEP:
                raise ValueError(
                    f"{name} step of {step} degrees is below the {MIN_RELIABLE_STEP} degree "
                    f"floor measured for this rig."
                )

        forward = True
        for tilt in _inclusive_range(tilt_start, tilt_end, tilt_step):
            pans = _inclusive_range(pan_start, pan_end, pan_step)
            if not forward:
                pans.reverse()
            forward = not forward

            for pan in pans:
                yield self.move_to(pan=pan, tilt=tilt)

    def verify_tilt(self, measured_pitch, tolerance=3.0):
        """Compares a measured IMU pitch against the commanded tilt.

        The IMU is bolted to this head, so its pitch independently measures the tilt axis --
        the only real position feedback on this rig. Persistent disagreement means the servo
        is not reaching its commanded angle. Requires the fixed offset between the IMU's zero
        and the tilt axis' zero to have been measured first.
        """
        if self._tilt is None:
            return False, None
        error = measured_pitch - self._tilt
        return abs(error) <= tolerance, error

    def close(self):
        self.release()
        self.pan_axis.close()
        self.tilt_axis.close()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()


def _inclusive_range(start, end, step):
    """Like range() for floats, and always includes the endpoint."""
    step = abs(step)
    if end < start:
        step = -step

    values = []
    value = start
    while (step > 0 and value <= end + 1e-9) or (step < 0 and value >= end - 1e-9):
        values.append(round(value, 3))
        value += step

    if values and abs(values[-1] - end) > 1e-9:
        values.append(end)
    return values


def main():
    parser = argparse.ArgumentParser(description="Drive and calibrate the pan/tilt gimbal.")
    parser.add_argument(
        "--backend", choices=("hwpwm", "gpiozero"), default="hwpwm",
        help="pulse source (default: hwpwm)"
    )
    parser.add_argument(
        "--pulse", type=float, metavar="US",
        help="drive the pan axis at a raw pulse width in microseconds and hold it. "
             "Use this to find where the servo actually hits its mechanical stops."
    )
    parser.add_argument(
        "--pulse-axis", choices=("pan", "tilt"), default="pan",
        help="which axis --pulse and --pulse-sweep drive (default: pan)"
    )
    parser.add_argument(
        "--pulse-sweep", action="store_true",
        help="walk pulse widths across the configured range, pausing at each, so you can "
             "watch where motion starts and stops"
    )
    parser.add_argument(
        "--step-test", type=float, metavar="DEGREES",
        help="repeatedly step the pan axis and print each commanded angle"
    )
    parser.add_argument(
        "--count", type=int, default=20,
        help="number of steps for --step-test (default: 20)"
    )
    parser.add_argument(
        "--detach", action="store_true",
        help="cut the pulse train after each move instead of holding position"
    )
    parser.add_argument(
        "--grid", action="store_true",
        help="walk a coarse scan grid and print each position"
    )
    args = parser.parse_args()

    with Gimbal(backend=args.backend, hold=not args.detach) as gimbal:
        axis = gimbal.pan_axis if args.pulse_axis == "pan" else gimbal.tilt_axis

        try:
            if args.pulse is not None:
                print(f"Holding {args.pulse_axis} at {args.pulse}us. Ctrl+C to stop.")
                axis.set_pulse_us(args.pulse)
                while True:
                    time.sleep(1)

            elif args.pulse_sweep:
                low, high = axis.pulse_limits
                print(f"Sweeping {args.pulse_axis} from {low}us to {high}us. "
                      f"Note where motion starts and stops.")
                for pulse in range(int(low), int(high) + 1, 100):
                    axis.set_pulse_us(pulse)
                    print(f"  {pulse:5d}us")
                    time.sleep(0.6)

            elif args.step_test is not None:
                # Deliberately bypasses MIN_RELIABLE_STEP: the point is to find the floor.
                print(f"Stepping pan by {args.step_test} degrees, {args.count} times.")
                gimbal.move_to(pan=90, tilt=90)
                for index in range(args.count):
                    pan, _ = gimbal.move_to(pan=90 + args.step_test * (index + 1))
                    print(f"  step {index + 1:3d} -> commanded pan {pan:7.2f}")

            elif args.grid:
                for pan, tilt in gimbal.scan_positions(0, 180, 30, 0, 90, 30):
                    print(f"  pan {pan:6.1f}  tilt {tilt:6.1f}")

            else:
                for pan in (0, 90, 180, 90):
                    gimbal.move_to(pan=pan)
                    print(f"  pan -> {gimbal.pan}")
                for tilt in (0, 45, 90):
                    gimbal.move_to(tilt=tilt)
                    print(f"  tilt -> {gimbal.tilt}")

        except KeyboardInterrupt:
            print("\nStopped by user.")


if __name__ == "__main__":
    main()
