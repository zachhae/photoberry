from pymavlink import mavutil
from picamera2 import Picamera2
import piexif
import time

master = mavutil.mavlink_connection('/dev/serial0', baud=921600)
master.wait_heartbeat()

print("Connected")

picam2 = Picamera2()
config = picam2.create_still_configuration()
picam2.configure(config)
picam2.start()
time.sleep(2)  # let auto-exposure/AF settle

def format_coords(val):
    # converts decimal degrees to rational tuple format (Deg/1, Min/1, Sec/1000)
    abs_val = abs(val)
    deg = int(abs_val)
    min_float = (abs_val - deg) * 60
    minute = int(min_float)
    sec = round((min_float - minute) * 60, 3)
    return ((deg, 1),(minute, 1),(int(sec*1000), 1000))

def capture_with_metadata(msg):
	# Capture image natively
    timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
    filename = f"Captures/capture_{timestamp}.jpg"
    picam2.capture_file(filename)
	
    # Scale raw data from CAMERA_FEEDBACK
    lat_deg = msg.lat / 1.0e7
    lon_deg = msg.lng / 1.0e7
    alt = abs(msg.alt_msl)
    
    # Determine reference directions
    lat_ref = 'N' if lat_deg >= 0 else 'S'
    lon_ref = 'E' if lon_deg >= 0 else 'W'

    # Prepare piexif dictionary
    gps_ifd = {
        piexif.GPSIFD.GPSLatitudeRef: lat_ref,
        piexif.GPSIFD.GPSLatitude: format_coords(lat_deg),
        piexif.GPSIFD.GPSLongitudeRef: lon_ref,
        piexif.GPSIFD.GPSLongitude: format_coords(lon_deg),
        piexif.GPSIFD.GPSAltitudeRef: 0, # above sea level
        piexif.GPSIFD.GPSAltitude: (int(alt*1000), 1000)
    }
    
    exif_dict = {"GPS": gps_ifd} # piexif expects specific IFD keys
    
    # Inject exif bytes into JPEG header
    exif_bytes = piexif.dump(exif_dict)
    piexif.insert(exif_bytes, filename)
    print(f"Captured & geotagged: {lat_deg:.6f}, {lon_deg:.6f}")

try:
	while True:
		msg = master.recv_match(type='CAMERA_FEEDBACK', blocking=True)
	
		if msg:
			capture_with_metadata(msg)
finally:
	picam2.close()
