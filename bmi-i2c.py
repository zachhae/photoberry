import argparse
import math
import struct
import time

from smbus2 import SMBus, i2c_msg

# Bus 1 is the Pi's primary I2C: GPIO2 (SDA, header pin 3) and GPIO3 (SCL, header pin 5).
# The BMI088 only speaks I2C when its PS pin (shuttle board P2-6) is tied to VDDIO.
I2C_BUS = 1

# The BMI088 is two dies in one package with separate addresses and separate register maps.
# Both maps start at 0x00, so the same register number means different things on each.
# Addresses assume the shuttle board's single SDO pin (P2-3) is tied to GND.
ACC_ADDR = 0x18
GYRO_ADDR = 0x68

# Accelerometer register map (Appendix 5.2/5.3 of the BMI088 data sheet)
ACC_CHIP_ID = 0x00  # reads 0x1E
ACC_ERR_REG = 0x02  # bit 0 fatal_err, bits 4:2 error_code
ACC_STATUS = 0x03  # bit 7 acc_drdy
ACC_DATA = 0x12  # 0x12-0x17 X/Y/Z, LSB first
ACC_CONF = 0x40  # bits 7:4 bandwidth (oversampling), bits 3:0 ODR
ACC_RANGE = 0x41  # 0x00 +/-3g, 0x01 +/-6g, 0x02 +/-12g, 0x03 +/-24g
ACC_PWR_CONF = 0x7C  # 0x00 active, 0x03 suspend (the reset default)
ACC_PWR_CTRL = 0x7D  # 0x00 accelerometer off, 0x04 on
ACC_SOFTRESET = 0x7E  # write 0xB6

# Gyroscope register map (Appendix 5.4/5.5)
GYRO_CHIP_ID = 0x00  # reads 0x0F
GYRO_DATA = 0x02  # 0x02-0x07 X/Y/Z rate, LSB first
GYRO_RANGE = 0x0F
GYRO_BANDWIDTH = 0x10  # ODR and low-pass filter as one setting
GYRO_LPM1 = 0x11  # 0x00 normal, 0x80 suspend, 0x20 deep suspend
GYRO_SOFTRESET = 0x14  # write 0xB6

ACC_CHIP_ID_VALUE = 0x1E
GYRO_CHIP_ID_VALUE = 0x0F

SOFTRESET_CMD = 0xB6

# Section 4.1: the accelerometer boots in suspend mode and produces nothing until it is
# explicitly switched on. The gyroscope is already running after power-up.
ACC_ENABLE = 0x04
ACC_ACTIVE = 0x00
GYRO_NORMAL_MODE = 0x00

# Timing, all from sections 3, 4.1 and 6.2. The BMI088 is considerably fussier than the
# TF-Luna, which tolerated a single blanket delay after every write.
ACC_BOOT_TIME = 0.001  # after power-on or soft-reset
ACC_NORMAL_SETTLE = 0.00045  # after writing ACC_PWR_CTRL
ACC_SOFTRESET_DELAY = 0.001
GYRO_MODE_CHANGE = 0.030  # any gyro power-mode switch; avoid bus traffic during it
GYRO_SOFTRESET_DELAY = 0.030
WRITE_SETTLE_NORMAL = 0.000002  # 2us between writes in normal mode
WRITE_SETTLE_SUSPEND = 0.00045  # 450us between writes while still in suspend

# ACC_RANGE register value -> full scale in g. The conversion in read_accelerometer() derives
# the scale factor from the register value directly, so this table is only for the CLI.
ACC_RANGES = {3: 0x00, 6: 0x01, 12: 0x02, 24: 0x03}

# ACC_CONF bits 3:0. Section 5.3.10.
ACC_ODRS = {12.5: 0x05, 25: 0x06, 50: 0x07, 100: 0x08,
            200: 0x09, 400: 0x0A, 800: 0x0B, 1600: 0x0C}

# ACC_CONF bits 7:4. "Normal" is the default; the oversampling modes trade bandwidth for
# noise. Section 4.4.1 has the resulting 3dB cutoffs.
ACC_BWP_NORMAL = 0x0A

# GYRO_RANGE register value -> full scale in degrees/second. Section 5.5.5.
GYRO_RANGES = {2000: 0x00, 1000: 0x01, 500: 0x02, 250: 0x03, 125: 0x04}

# GYRO_BANDWIDTH register value -> (ODR Hz, filter bandwidth Hz). Section 5.5.6.
GYRO_BANDWIDTHS = {
    0x00: (2000, 532), 0x01: (2000, 230), 0x02: (1000, 116), 0x03: (400, 47),
    0x04: (200, 23), 0x05: (100, 12), 0x06: (200, 64), 0x07: (100, 32),
}

# Complementary filter weight. Higher trusts the gyroscope more over short intervals; the
# remainder pulls roll and pitch back toward what gravity says so they cannot drift.
DEFAULT_ALPHA = 0.98

# The accelerometer measures gravity plus real motion and cannot tell them apart, so its
# angle estimate is only meaningful when the total acceleration is close to 1g. Outside this
# band the sample is dynamic and the filter coasts on the gyroscope alone.
STATIC_G_MIN = 0.85
STATIC_G_MAX = 1.15

# The register interface has no checksums, so a disturbed bus surfaces as an OSError from the
# ioctl rather than as a value that can be validated. A single dropped transaction is not a
# reason to lose the run, so reads retry briefly and the loop tolerates a short burst of
# failures before giving up.
I2C_RETRIES = 3
I2C_RETRY_DELAY = 0.002
MAX_CONSECUTIVE_ERRORS = 10

bus = SMBus(I2C_BUS)


def read_registers(addr, reg, length):
    """Reads `length` bytes starting at register `reg` from one of the two dies.

    Section 6.2 specifies the read as a write phase and a read phase separated by a repeated
    start, so both messages go into a single i2c_rdwr call. This differs from tf-i2c.py, where
    the TF-Luna's timing diagram called for two independent transactions.

    Note that the dummy-byte quirk in section 6.1.2 applies only to SPI reads of the
    accelerometer. Over I2C there is no dummy byte; drivers that skip one here end up off by
    one byte on every axis.
    """
    for attempt in range(I2C_RETRIES):
        try:
            write = i2c_msg.write(addr, [reg])
            read = i2c_msg.read(addr, length)
            bus.i2c_rdwr(write, read)
            return bytes(read)
        except OSError:
            # Almost always a disturbed bus rather than a sick sensor: a nudged jumper, a
            # marginal contact, or noise on a long run. Retrying costs microseconds.
            if attempt == I2C_RETRIES - 1:
                raise
            time.sleep(I2C_RETRY_DELAY)


def write_register(addr, reg, value, settle=WRITE_SETTLE_NORMAL):
    """Writes one byte and waits for the sensor to process it."""
    bus.i2c_rdwr(i2c_msg.write(addr, [reg, value]))
    time.sleep(settle)


def check_chip_ids():
    """Confirms both dies are present and responding before anything else runs.

    Same purpose as the "LUNA" signature check in tf-i2c.py: turn a wiring mistake into a
    clear error instead of plausible-looking garbage.
    """
    acc_id = read_registers(ACC_ADDR, ACC_CHIP_ID, 1)[0]
    if acc_id != ACC_CHIP_ID_VALUE:
        raise RuntimeError(
            f"Accelerometer at 0x{ACC_ADDR:02X} returned chip ID 0x{acc_id:02X}, "
            f"expected 0x{ACC_CHIP_ID_VALUE:02X}."
        )

    gyro_id = read_registers(GYRO_ADDR, GYRO_CHIP_ID, 1)[0]
    if gyro_id != GYRO_CHIP_ID_VALUE:
        raise RuntimeError(
            f"Gyroscope at 0x{GYRO_ADDR:02X} returned chip ID 0x{gyro_id:02X}, "
            f"expected 0x{GYRO_CHIP_ID_VALUE:02X}."
        )

    print(f"Found BMI088: accelerometer at 0x{ACC_ADDR:02X}, gyroscope at 0x{GYRO_ADDR:02X}")


def check_accelerometer_errors():
    """Reads ACC_ERR_REG and reports any persistent fault (section 5.3.2)."""
    err = read_registers(ACC_ADDR, ACC_ERR_REG, 1)[0]
    error_code = (err >> 2) & 0x07
    fatal = err & 0x01

    if fatal:
        raise RuntimeError(
            "Accelerometer reports fatal_err: the chip is not in an operational state. "
            "Only a power-on or soft-reset clears this."
        )
    if error_code == 0x01:
        raise RuntimeError(
            "Accelerometer reports an invalid ACC_CONF setting. Check the ODR and "
            "bandwidth combination."
        )
    if error_code:
        print(f" -> WARNING: ACC_ERR_REG reports error_code 0x{error_code:02X}")


def soft_reset():
    """Resets both dies to their power-on defaults.

    Optional. Section 4.8.1 notes the accelerometer releases SDA immediately on reset, which
    some hosts see as a missing ACK, so this stays behind a flag rather than running by default.
    """
    print("Soft-resetting both sensors...")
    write_register(ACC_ADDR, ACC_SOFTRESET, SOFTRESET_CMD, settle=ACC_SOFTRESET_DELAY)
    write_register(GYRO_ADDR, GYRO_SOFTRESET, SOFTRESET_CMD, settle=GYRO_SOFTRESET_DELAY)


def configure_accelerometer(range_g, odr):
    """Wakes the accelerometer and sets its range and output data rate."""
    print(" -> Waking the accelerometer")
    # Section 4.1.1: after any reset the accelerometer sits in suspend mode and returns
    # nothing at all. Writes here are still subject to the 450us suspend-mode spacing.
    time.sleep(ACC_BOOT_TIME)
    write_register(ACC_ADDR, ACC_PWR_CONF, ACC_ACTIVE, settle=WRITE_SETTLE_SUSPEND)
    write_register(ACC_ADDR, ACC_PWR_CTRL, ACC_ENABLE, settle=ACC_NORMAL_SETTLE)

    print(f" -> Accelerometer range +/-{range_g}g at {odr}Hz")
    write_register(ACC_ADDR, ACC_CONF, (ACC_BWP_NORMAL << 4) | ACC_ODRS[odr])
    write_register(ACC_ADDR, ACC_RANGE, ACC_RANGES[range_g])

    # A bad ODR/bandwidth pair shows up here rather than as strange readings later.
    check_accelerometer_errors()


def configure_gyroscope(range_dps, bw):
    """Puts the gyroscope in normal mode and sets its range and bandwidth."""
    odr, filter_bw = GYRO_BANDWIDTHS[bw]

    print(" -> Setting gyroscope to normal mode")
    # Already normal after power-up, but writing it makes the state deterministic. Section
    # 4.1.2 wants 30ms of silence after any power-mode write.
    write_register(GYRO_ADDR, GYRO_LPM1, GYRO_NORMAL_MODE, settle=GYRO_MODE_CHANGE)

    print(f" -> Gyroscope range +/-{range_dps} deg/s at {odr}Hz (filter {filter_bw}Hz)")
    write_register(GYRO_ADDR, GYRO_RANGE, GYRO_RANGES[range_dps])
    write_register(GYRO_ADDR, GYRO_BANDWIDTH, bw)


def read_accelerometer(range_reg):
    """Reads all three axes as one coherent 6-byte block, in g."""
    # Section 4.2: reading an LSB register locks its MSB until read, and a burst read locks
    # the whole set, so the three axes are guaranteed to come from the same sample.
    data = read_registers(ACC_ADDR, ACC_DATA, 6)
    x, y, z = struct.unpack('<hhh', data)

    # Section 5.3.4: mg = raw / 32768 * 1000 * 2^(ACC_RANGE + 1) * 1.5, divided by 1000 for g.
    scale = 2 ** (range_reg + 1) * 1.5 / 32768.0
    return x * scale, y * scale, z * scale


def read_gyroscope(full_scale):
    """Reads all three axes as one coherent 6-byte block, in degrees/second."""
    data = read_registers(GYRO_ADDR, GYRO_DATA, 6)
    x, y, z = struct.unpack('<hhh', data)

    # Section 5.5.2: the full scale spans the signed 16-bit range.
    scale = full_scale / 32768.0
    return x * scale, y * scale, z * scale


def calibrate_gyroscope(full_scale, samples, interval):
    """Averages the gyroscope at rest to find its zero-rate offset.

    Section 1.3 gives a typical zero-rate offset of +/-1 deg/s. Integrated over a minute that
    is roughly 60 degrees of accumulated error, so this is a required step rather than a
    refinement.
    """
    if not samples:
        print(" -> Skipping gyroscope calibration (bias assumed zero)")
        return 0.0, 0.0, 0.0

    print(f" -> Calibrating gyroscope over {samples} samples, hold the sensor still...")
    totals = [0.0, 0.0, 0.0]
    for _ in range(samples):
        rates = read_gyroscope(full_scale)
        for axis in range(3):
            totals[axis] += rates[axis]
        time.sleep(interval)

    bias = tuple(total / samples for total in totals)
    print(f"    bias: x={bias[0]:+.3f} y={bias[1]:+.3f} z={bias[2]:+.3f} deg/s")

    if max(abs(value) for value in bias) > 5.0:
        print("    WARNING: bias is unusually large, the sensor may have been moving")

    return bias


class Orientation:
    """Fuses accelerometer and gyroscope into roll, pitch and yaw.

    Roll and pitch come from a complementary filter: the gyroscope supplies smooth short-term
    motion, and gravity continuously pulls the estimate back so it cannot drift.

    Yaw is gyroscope-only. With no magnetometer the BMI088 has no absolute heading reference,
    so yaw drifts without bound and is reported for reference only. For the scanning rig, take
    the pan angle from the commanded servo position instead.
    """

    def __init__(self, alpha=DEFAULT_ALPHA):
        self.alpha = alpha
        self.roll = None
        self.pitch = None
        self.yaw = 0.0

    @staticmethod
    def _from_gravity(ax, ay, az):
        """Roll and pitch implied by the gravity vector alone, in degrees."""
        roll = math.degrees(math.atan2(ay, az))
        pitch = math.degrees(math.atan2(-ax, math.hypot(ay, az)))
        return roll, pitch

    def update(self, accel, rates, dt):
        """Advances the estimate by one sample. Returns (roll, pitch, yaw, static)."""
        ax, ay, az = accel
        gx, gy, gz = rates

        accel_roll, accel_pitch = self._from_gravity(ax, ay, az)

        # Seed from gravity on the first sample so the filter starts settled rather than
        # sweeping in from zero.
        if self.roll is None:
            self.roll, self.pitch = accel_roll, accel_pitch
            return self.roll, self.pitch, self.yaw, True

        magnitude = math.sqrt(ax * ax + ay * ay + az * az)
        static = STATIC_G_MIN <= magnitude <= STATIC_G_MAX

        roll = self.roll + gx * dt
        pitch = self.pitch + gy * dt
        if static:
            # Only trust gravity when the sensor is not being accelerated; otherwise the
            # "down" it reports is a mix of gravity and motion.
            roll = self.alpha * roll + (1.0 - self.alpha) * accel_roll
            pitch = self.alpha * pitch + (1.0 - self.alpha) * accel_pitch

        self.roll, self.pitch = roll, pitch
        self.yaw += gz * dt

        return self.roll, self.pitch, self.yaw, static


def read_imu(range_reg, full_scale, interval, bias, alpha, print_every):
    """Polls both sensors and prints fused readings until interrupted."""
    print("\nReading IMU data (Press Ctrl+C to stop)...")
    orientation = Orientation(alpha)
    bias_x, bias_y, bias_z = bias
    previous = time.time()
    sample = 0
    consecutive_errors = 0
    total_errors = 0

    try:
        while True:
            time.sleep(interval)

            now = time.time()

            try:
                accel = read_accelerometer(range_reg)
                raw_rates = read_gyroscope(full_scale)
            except OSError as exc:
                consecutive_errors += 1
                total_errors += 1
                print(f"[{now:.4f}s] I2C read failed ({exc}) "
                      f"- {consecutive_errors} in a row")
                if consecutive_errors >= MAX_CONSECUTIVE_ERRORS:
                    print("Too many consecutive I2C failures, stopping. Check the wiring.")
                    break
                # Leave `previous` alone so the next good sample integrates across the gap
                # instead of under-counting the elapsed time.
                continue

            consecutive_errors = 0
            dt = now - previous
            previous = now

            rates = (raw_rates[0] - bias_x, raw_rates[1] - bias_y, raw_rates[2] - bias_z)

            roll, pitch, yaw, static = orientation.update(accel, rates, dt)

            # The filter runs on every sample; printing is decimated so the output stays
            # readable at high output data rates.
            sample += 1
            if sample % print_every:
                continue

            # Flags a sample where the accelerometer was excluded from the fusion, the same
            # way tf-i2c.py flags a range reading its amplitude says not to trust.
            note = "" if static else " | DYNAMIC"
            print(
                f"[{now:.4f}s] "
                f"Acc: {accel[0]:+6.3f} {accel[1]:+6.3f} {accel[2]:+6.3f} g | "
                f"Gyro: {rates[0]:+7.2f} {rates[1]:+7.2f} {rates[2]:+7.2f} deg/s | "
                f"Roll: {roll:+7.2f} Pitch: {pitch:+7.2f} Yaw: {yaw:+8.2f}{note}"
            )

    except KeyboardInterrupt:
        print("\nStopped by user.")

    if total_errors:
        print(f"Recovered from {total_errors} I2C read failure(s) during the run.")


def main():
    parser = argparse.ArgumentParser(
        description="Read a BMI088 IMU over I2C on a Raspberry Pi and fuse it into orientation."
    )
    parser.add_argument(
        "--acc-range", type=int, default=6, choices=sorted(ACC_RANGES),
        help="accelerometer full scale in g (default: 6)"
    )
    parser.add_argument(
        "--acc-odr", type=float, default=100, choices=sorted(ACC_ODRS),
        help="accelerometer output data rate in Hz; also paces the loop (default: 100)"
    )
    parser.add_argument(
        "--gyro-range", type=int, default=1000, choices=sorted(GYRO_RANGES),
        # The IMU rides on the pan/tilt head, so it sees the servos' own rotation. An MG90S
        # slews at roughly 600 deg/s when commanded straight to an angle, which would clip
        # +/-500. Drop to 500 or 250 for finer resolution if the sensor is ever moved to the
        # base, where servo speed no longer applies.
        help="gyroscope full scale in deg/s; smaller is finer resolution (default: 1000)"
    )
    parser.add_argument(
        "--gyro-bw", type=lambda v: int(v, 0), default=0x04, choices=sorted(GYRO_BANDWIDTHS),
        help="GYRO_BANDWIDTH register value 0-7 selecting ODR and filter (default: 4 = 200Hz/23Hz)"
    )
    parser.add_argument(
        "--calibration-samples", type=int, default=500,
        help="gyroscope bias samples to average at rest; 0 disables (default: 500)"
    )
    parser.add_argument(
        "--alpha", type=float, default=DEFAULT_ALPHA,
        help="complementary filter weight on the gyroscope, 0-1 (default: 0.98)"
    )
    parser.add_argument(
        "--print-every", type=int, default=10,
        help="print one line per N samples; the filter still runs on all of them (default: 10)"
    )
    parser.add_argument(
        "--reset", action="store_true",
        help="soft-reset both sensors before configuring them"
    )
    args = parser.parse_args()

    if not 0.0 <= args.alpha <= 1.0:
        parser.error("--alpha must be between 0 and 1")
    if args.print_every < 1:
        parser.error("--print-every must be at least 1")

    interval = 1.0 / args.acc_odr
    range_reg = ACC_RANGES[args.acc_range]
    full_scale = args.gyro_range

    try:
        check_chip_ids()

        if args.reset:
            soft_reset()

        print("Configuring BMI088...")
        configure_accelerometer(args.acc_range, args.acc_odr)
        configure_gyroscope(args.gyro_range, args.gyro_bw)

        bias = calibrate_gyroscope(full_scale, args.calibration_samples, interval)
        print("Configuration complete!")

        read_imu(range_reg, full_scale, interval, bias, args.alpha, args.print_every)
    finally:
        bus.close()
        print("I2C bus closed.")


if __name__ == "__main__":
    main()
