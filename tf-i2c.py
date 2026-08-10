import argparse
import struct
import time

from smbus2 import SMBus, i2c_msg

# Bus 1 is the Pi's primary I2C: GPIO2 (SDA, header pin 3) and GPIO3 (SCL, header pin 5).
# The TF-Luna only speaks I2C when its pin 5 (configuration input) is tied to GND.
I2C_BUS = 1

# Default 7-bit slave address.
ADDR = 0x10

# Register map (Appendix III of the TF-Luna user manual)
REG_DIST_LOW = 0x00  # 0x00-0x01 distance, centimeters
REG_AMP_LOW = 0x02  # 0x02-0x03 signal amplitude
REG_TEMP_LOW = 0x04  # 0x04-0x05 chip temperature, unit 0.01 Celsius
REG_TICK_LOW = 0x06  # 0x06-0x07 timestamp
REG_ERROR_LOW = 0x08  # 0x08-0x09 error code
REG_VERSION_REVISION = 0x0A  # 0x0A-0x0C revision, minor, major
REG_SAVE = 0x20  # write 0x01 to persist the current settings
REG_REBOOT = 0x21  # write 0x02 to reboot
REG_SLAVE_ADDR = 0x22
REG_MODE = 0x23  # 0x00 continuous ranging, 0x01 trigger
REG_TRIG_ONE_SHOT = 0x24  # write 0x01 to take one measurement (trigger mode only)
REG_ENABLE = 0x25  # 0x00 LiDAR off, 0x01 LiDAR on
REG_FPS_LOW = 0x26  # 0x26-0x27 output frequency, Hz
REG_SIGNATURE = 0x3C  # 0x3C-0x3F reads back "LUNA"

MODE_CONTINUOUS = 0x00
MODE_TRIGGER = 0x01

# Section 6.3: a write needs time to process, so wait 100ms before reading the register back.
WRITE_SETTLE = 0.1

# Pin 6 is the data-ready signal in I2C mode: high when a new sample lands, low again after any
# register read. Set this to a GPIO pin number if that wire is connected, otherwise leave it None
# and the loop free-runs off the output frequency instead.
READY_PIN = 4

bus = SMBus(I2C_BUS)


def read_registers(reg, length):
    """Reads `length` bytes starting at register `reg`.

    Section 6.3 documents the read as two transactions: a write that selects the register,
    then a separate read. A repeated start in place of the stop is also allowed, but this
    matches the timing diagram exactly and works with the widest range of firmware.
    """
    bus.i2c_rdwr(i2c_msg.write(ADDR, [reg]))
    read = i2c_msg.read(ADDR, length)
    bus.i2c_rdwr(read)
    return bytes(read)


def write_register(reg, value, settle=WRITE_SETTLE):
    """Writes one byte to `reg` and waits for the sensor to process it."""
    bus.i2c_rdwr(i2c_msg.write(ADDR, [reg, value]))
    time.sleep(settle)


def check_signature():
    """Confirms we are actually talking to a TF-Luna and prints its firmware version."""
    signature = read_registers(REG_SIGNATURE, 4)
    if signature != b'LUNA':
        raise RuntimeError(
            f"Signature register returned {signature!r}, expected b'LUNA'. "
            f"Check that pin 5 is grounded and the sensor is at address 0x{ADDR:02X}."
        )

    revision, minor, major = read_registers(REG_VERSION_REVISION, 3)
    print(f"Found TF-Luna at 0x{ADDR:02X}, firmware v{major}.{minor}.{revision}")

    # I2C support landed in V1.0.7; older firmware will answer erratically or not at all.
    if (major, minor, revision) < (1, 0, 7):
        print(" -> WARNING: firmware is below v1.0.7, I2C behavior is not guaranteed")


def configure_sensor(fps, trigger, save):
    """Turns the LiDAR on and puts it in continuous or trigger mode."""
    print("Configuring TF-Luna...")

    print(" -> Turning LiDAR on")
    write_register(REG_ENABLE, 0x01)

    if trigger:
        print(" -> Setting trigger mode")
        write_register(REG_MODE, MODE_TRIGGER)
    else:
        print(f" -> Setting continuous ranging mode at {fps}Hz")
        write_register(REG_MODE, MODE_CONTINUOUS)
        # Frequency is a little-endian 16-bit value split across two registers.
        write_register(REG_FPS_LOW, fps & 0xFF)
        write_register(REG_FPS_LOW + 1, (fps >> 8) & 0xFF)

    if save:
        print(" -> Saving current settings")
        write_register(REG_SAVE, 0x01)

    print("Configuration complete!\n")


def read_frame():
    """Reads distance, amplitude, and temperature as one coherent 6-byte block."""
    # One transaction keeps the three values from straddling a sensor update.
    data = read_registers(REG_DIST_LOW, 6)
    dist, amp, temp_raw = struct.unpack('<HHh', data)

    # Over I2C the temperature register is already scaled to hundredths of a degree,
    # unlike the serial protocol's Temp/8 - 256 conversion.
    return dist, amp, temp_raw / 100.0


def read_lidar(fps, trigger, ready):
    """Polls the sensor and prints each measurement until interrupted."""
    if trigger:
        print("Reading LiDAR data on demand (Press Enter to measure, Ctrl+C to stop)...")
    else:
        print("Reading LiDAR data (Press Ctrl+C to stop)...")
    interval = 1.0 / fps

    try:
        while True:
            if trigger:
                # Nothing is ranged until the one-shot register is written, so the loop
                # can wait here indefinitely without the sensor falling behind.
                input("> ")
                # Short settle here rather than the full 100ms: we are not reading the
                # trigger register back, just giving the sensor time to range.
                write_register(REG_TRIG_ONE_SHOT, 0x01, settle=0.01)
            elif ready is not None:
                # Section 6.3: in continuous mode, read only while pin 6 is high, otherwise
                # the read can collide with the sensor updating its data registers.
                if not ready.wait_for_active(timeout=1.0):
                    print("Timed out waiting for data-ready on pin 6")
                    continue
            else:
                time.sleep(interval)

            system_timestamp = time.time()
            dist, amp, temperature = read_frame()

            # Amp < 100 or Amp == 65535 indicates low signal or overexposure
            if amp < 100 or amp == 0xFFFF:
                print(f"[{system_timestamp:.4f}s] Dist: UNRELIABLE | Amp: {amp}")
                continue

            # Above 32768 the sensor is looking at something like direct sunlight
            note = " | AMBIENT OVEREXPOSURE" if amp > 32768 else ""
            print(
                f"[{system_timestamp:.4f}s] Dist: {dist} cm | Amp: {amp} | "
                f"Temp: {temperature:.1f}°C{note}"
            )

    except KeyboardInterrupt:
        print("\nStopped by user.")


def main():
    parser = argparse.ArgumentParser(description="Read a TF-Luna over I2C on a Raspberry Pi.")
    parser.add_argument(
        "--fps", type=int, default=100,
        help="output frequency in Hz for continuous mode; must be 500/n and <= 250 (default: 100)"
    )
    parser.add_argument(
        "--trigger", action="store_true",
        help="use trigger mode (one measurement per request) instead of continuous ranging"
    )
    parser.add_argument(
        "--save", action="store_true",
        help="persist the configuration to the sensor so it survives a power cycle"
    )
    args = parser.parse_args()

    ready = None
    if READY_PIN is not None and not args.trigger:
        from gpiozero import DigitalInputDevice
        ready = DigitalInputDevice(READY_PIN)

    try:
        check_signature()
        configure_sensor(args.fps, args.trigger, args.save)
        read_lidar(args.fps, args.trigger, ready)
    finally:
        bus.close()
        print("I2C bus closed.")


if __name__ == "__main__":
    main()
