from adafruit_vl6180x import VL6180X
from .i2c_mux import select_channel

def initialize_vl6180x(i2c, channels):
    sensors_vl6180x = {}
    for channel in channels:
        select_channel(i2c, channel)
        try:
            sensor = VL6180X(i2c)
            sensor.range  # check if work
            sensors_vl6180x[channel] = sensor
            print(f"VL6180X detected on channel {channel}")
        except (ValueError, OSError, RuntimeError):
            print(f"No VL6180X on channel {channel}")
    return sensors_vl6180x