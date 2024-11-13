from adafruit_vl53l1x import VL53L1X
from .i2c_mux import select_channel

def initialize_vl53l1x(i2c, channels):
    sensors_vl53l1x = {}
    for channel in channels:
        select_channel(i2c, channel)
        try:
            sensor = VL53L1X(i2c)
            sensor.start_ranging() 
            # Try read range to check sensor working
            sensor.distance
            sensors_vl53l1x[channel] = sensor
            print(f"VL53L1X detected on channel {channel}")
        except (ValueError, OSError, RuntimeError):
            print(f"No VL53L1X on channel {channel}")
    return sensors_vl53l1x