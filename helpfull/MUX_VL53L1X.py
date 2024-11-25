import time
import board
import busio
from adafruit_vl53l1x import VL53L1X

i2c = busio.I2C(board.SCL, board.SDA)

MUX_ADDRESS = 0x70

def select_channel(i2c, channel):
    if 0 <= channel <= 7:  
        i2c.writeto(MUX_ADDRESS, bytes([1 << channel]))
    else:
        raise ValueError("Channel must be between 0 and 7")

# Try init
def initialize_sensor_on_channel(i2c, channel):
    select_channel(i2c, channel)  
    try:
        sensor = VL53L1X(i2c)
        sensor.start_ranging() 
        # Try read range to check sensor working
        sensor.distance
        return sensor
    except (ValueError, OSError, RuntimeError):
        # Ignor errors when sensor isn't on channel
        return None

channels = list(range(8))  
sensors = {}  

for channel in channels:
    sensor = initialize_sensor_on_channel(i2c, channel)
    if sensor:
        sensors[channel] = sensor  
        print(f"Sensor detected on channel {channel}")
    else:
        print(f"No sensor detected on channel {channel}")

try:
    while True:
        for channel, sensor in sensors.items():
            select_channel(i2c, channel)  
            try:
                distance = sensor.distance 
                print(f"Channel {channel}, Distance: {distance} cm")
            except RuntimeError:
                print(f"Channel {channel}, Error reading distance")
        
        time.sleep(1)

except KeyboardInterrupt:
    print("Program interrupted")

finally:
    for sensor in sensors.values():
        sensor.stop_ranging()
