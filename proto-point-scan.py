"""Prototype hemisphere scan: pan/tilt gimbal + TF-Luna + BMI088 -> point cloud.

Stop-and-shoot. The head moves to a position, waits until the gyroscope confirms it has
actually stopped, averages several range readings, and records one point. Nothing is measured
while the head is in motion, which sidesteps the whole problem of aligning three sensors'
timestamps -- when everything is stationary it does not matter that the readings arrived a few
milliseconds apart.

Scan path: for every pan step, sweep tilt from straight up down past horizontal, then return
tilt to the top before stepping pan. Always approaching a tilt angle from the same direction
also means gear backlash biases every point the same way instead of alternating.

Outputs two files per run:
  scan-<timestamp>.ply  vertices only, for CloudCompare / MeshLab / Blender
  scan-<timestamp>.csv  every measurement plus the raw angles and IMU attitude, for
                        reprocessing later without rescanning
"""

import importlib.util
import math
import os
import time

import gimbal

# --- Scan pattern -----------------------------------------------------------------------
PAN_STEP = 3.0  # degrees
TILT_STEP = 3.0

# Range readings averaged at each position. The head is stationary, so this is free accuracy.
SAMPLES_PER_POINT = 5

# The gyroscope is the settle detector: below this total rotation rate the head is considered
# stopped. Well above the post-calibration noise floor, well below any real motion.
STILL_THRESHOLD = 2.0  # deg/s
SETTLE_TIMEOUT = 1.0  # seconds to wait for stillness before measuring anyway

# TF-Luna amplitude below this means the range is not trustworthy (TF-Luna manual 6.3).
MIN_AMPLITUDE = 100
MAX_AMPLITUDE = 0xFFFF


def load_module(name, filename):
    """Imports a module whose filename contains a hyphen.

    tf-i2c.py and bmi-i2c.py cannot be imported normally because a hyphen is not valid in an
    identifier. Loading them by path keeps all the register-level knowledge and datasheet
    commentary in one place instead of copying it here.
    """
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), filename)
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


tf = load_module("tf_i2c", "tf-i2c.py")
bmi = load_module("bmi_i2c", "bmi-i2c.py")

# Sensor settings, matching each driver's own defaults.
LUNA_FPS = 100
ACC_RANGE_G = 6
ACC_ODR_HZ = 100
GYRO_RANGE_DPS = 1000
GYRO_BW = 0x04


def spherical_to_cartesian(distance_m, pan_deg, tilt_deg):
    """Places one range reading in space.

    Tilt is measured from straight up, which makes it exactly the polar angle of the physics
    spherical convention, and pan is the azimuth. So the usual conversion applies directly:

        x = d * sin(tilt) * cos(pan)
        y = d * sin(tilt) * sin(pan)
        z = d * cos(tilt)

    Straight up (tilt 0) gives a point directly overhead, horizontal (tilt 90) gives one at
    z = 0, and past horizontal gives a negative z. Units are metres, matching the file output.
    """
    pan = math.radians(pan_deg)
    tilt = math.radians(tilt_deg)

    x = distance_m * math.sin(tilt) * math.cos(pan)
    y = distance_m * math.sin(tilt) * math.sin(pan)
    z = distance_m * math.cos(tilt)
    return x, y, z


def wait_until_still(full_scale, bias):
    """Blocks until the gyroscope says the head has stopped, or the timeout expires.

    A closed-loop settle check. The fixed delay in gimbal.move_to() is a conservative
    estimate; this confirms it. Returns True if the head actually settled.
    """
    bias_x, bias_y, bias_z = bias
    deadline = time.time() + SETTLE_TIMEOUT

    while time.time() < deadline:
        x, y, z = bmi.read_gyroscope(full_scale)
        rate = math.sqrt((x - bias_x) ** 2 + (y - bias_y) ** 2 + (z - bias_z) ** 2)
        if rate < STILL_THRESHOLD:
            return True
        time.sleep(0.02)

    return False


def measure_range():
    """Averages several TF-Luna readings, discarding the ones it flags as unreliable.

    Returns (distance_cm, amplitude) or (None, amplitude) if nothing usable came back.
    """
    distances = []
    amplitudes = []

    for _ in range(SAMPLES_PER_POINT):
        distance, amplitude, _ = tf.read_frame()
        if MIN_AMPLITUDE <= amplitude < MAX_AMPLITUDE and distance > 0:
            distances.append(distance)
            amplitudes.append(amplitude)
        time.sleep(1.0 / LUNA_FPS)

    if not distances:
        return None, amplitudes[0] if amplitudes else 0

    return sum(distances) / len(distances), sum(amplitudes) / len(amplitudes)


def read_attitude(range_reg):
    """Roll and pitch of the head from gravity alone.

    No complementary filter here. The filter in bmi-i2c.py exists to survive motion, but the
    head is deliberately stationary at every measurement, so the gravity vector on its own is
    both simpler and more accurate. Returns (roll, pitch, magnitude_g).
    """
    ax, ay, az = bmi.read_accelerometer(range_reg)
    roll, pitch = bmi.Orientation._from_gravity(ax, ay, az)
    magnitude = math.sqrt(ax * ax + ay * ay + az * az)
    return roll, pitch, magnitude


def write_ply(path, points):
    """Writes an ASCII PLY containing just vertices."""
    with open(path, "w") as handle:
        handle.write("ply\n")
        handle.write("format ascii 1.0\n")
        handle.write(f"element vertex {len(points)}\n")
        handle.write("property float x\n")
        handle.write("property float y\n")
        handle.write("property float z\n")
        handle.write("end_header\n")
        for x, y, z in points:
            handle.write(f"{x:.4f} {y:.4f} {z:.4f}\n")


def main():
    stamp = time.strftime("%Y%m%d-%H%M%S")
    ply_path = f"scan-{stamp}.ply"
    csv_path = f"scan-{stamp}.csv"

    pan_angles = gimbal._inclusive_range(
        gimbal.PAN_LIMITS[0], gimbal.PAN_LIMITS[1], PAN_STEP)
    tilt_angles = gimbal._inclusive_range(
        gimbal.TILT_LIMITS[0], gimbal.TILT_LIMITS[1], TILT_STEP)

    total = len(pan_angles) * len(tilt_angles)
    print(f"Scan grid: {len(pan_angles)} pan x {len(tilt_angles)} tilt = {total} points")
    print(f"Writing {ply_path} and {csv_path}\n")

    print("Starting TF-Luna...")
    tf.check_signature()
    tf.configure_sensor(LUNA_FPS, trigger=False, save=False)

    print("Starting BMI088...")
    bmi.check_chip_ids()
    bmi.configure_accelerometer(ACC_RANGE_G, ACC_ODR_HZ)
    bmi.configure_gyroscope(GYRO_RANGE_DPS, GYRO_BW)
    bias = bmi.calibrate_gyroscope(GYRO_RANGE_DPS, 500, 1.0 / ACC_ODR_HZ)
    acc_range_reg = bmi.ACC_RANGES[ACC_RANGE_G]

    points = []
    skipped = 0
    started = time.time()

    csv_file = open(csv_path, "w")
    csv_file.write("index,pan_deg,tilt_deg,distance_m,amplitude,"
                   "x,y,z,imu_roll,imu_pitch,settled\n")

    try:
        with gimbal.Gimbal() as head:
            for pan in pan_angles:
                # Return to the top of the tilt sweep before every pan step so each column is
                # scanned in the same direction.
                head.move_to(pan=pan, tilt=tilt_angles[0])

                for tilt in tilt_angles:
                    head.move_to(pan=pan, tilt=tilt)
                    settled = wait_until_still(GYRO_RANGE_DPS, bias)

                    roll, pitch, magnitude = read_attitude(acc_range_reg)
                    distance_cm, amplitude = measure_range()

                    index = len(points) + skipped

                    if distance_cm is None:
                        skipped += 1
                        print(f"[{index:6d}] pan {pan:6.1f} tilt {tilt:6.1f} "
                              f"| no valid return (amp {amplitude:.0f})")
                        continue

                    distance_m = distance_cm / 100.0
                    x, y, z = spherical_to_cartesian(distance_m, pan, tilt)
                    points.append((x, y, z))

                    flag = "" if settled else " | NOT SETTLED"
                    print(f"[{index:6d}] pan {pan:6.1f} tilt {tilt:6.1f} "
                          f"| d {distance_m:6.3f} m amp {amplitude:6.0f} "
                          f"| xyz {x:+7.3f} {y:+7.3f} {z:+7.3f} "
                          f"| roll {roll:+6.1f} pitch {pitch:+6.1f}{flag}")

                    csv_file.write(
                        f"{index},{pan:.1f},{tilt:.1f},{distance_m:.4f},{amplitude:.0f},"
                        f"{x:.4f},{y:.4f},{z:.4f},{roll:.2f},{pitch:.2f},{int(settled)}\n"
                    )
                    csv_file.flush()

    except KeyboardInterrupt:
        print("\nStopped by user, writing what we have...")

    finally:
        csv_file.close()
        write_ply(ply_path, points)

        elapsed = time.time() - started
        print(f"\n{len(points)} points written to {ply_path}")
        print(f"{skipped} positions returned nothing usable")
        print(f"Elapsed {elapsed / 60:.1f} minutes")

        tf.bus.close()
        bmi.bus.close()
        print("I2C buses closed.")


if __name__ == "__main__":
    main()
