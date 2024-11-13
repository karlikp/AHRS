import sys
import os
import time
import board
import busio
from smbus2 import SMBus  

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__),'AHRS_package')))
from AHRS_package import initialize_all_sensors, select_channel, read_bmx160

i2c = busio.I2C(board.SCL, board.SDA)
bus = SMBus(1) 


#Initialize
sensors_vl6180x, sensors_vl53l1x, bmp388_sensor, bmx160_initialized = initialize_all_sensors(i2c, bus)

try:
    while True:
        # Read BMP388
        if bmp388_sensor:
            select_channel(i2c, 0)
            try:
                temperature = bmp388_sensor.temperature
                pressure = bmp388_sensor.pressure
                print(f"Channel 0, BMP388 Temperature: {temperature:.2f} C, Pressure: {pressure:.2f} hPa")
            except RuntimeError:
                print("BMP388 Error reading data")

        # Read BMX160
        if bmx160_initialized:
            select_channel(i2c, 0)
            try:
                accel_x, accel_y, accel_z = read_bmx160(bus)
                if accel_x is not None:
                    print(f"Channel 0, BMX160 Accel X: {accel_x}, Accel Y: {accel_y}, Accel Z: {accel_z}")
                else:
                    print("BMX160 Error reading data")
            except RuntimeError:
                print("BMX160 Error reading data")
                
        # Read VL6180X
        for channel, sensor in sensors_vl6180x.items():
            select_channel(i2c, channel)
            try:
                distance = sensor.range
                print(f"Channel {channel}, VL6180X Distance: {distance} mm")
            except RuntimeError:
                print(f"Channel {channel}, VL6180X Error reading distance")
        
        for channel, sensor in sensors_vl53l1x.items():
            select_channel(i2c, channel)
            try:
                distance = sensor.distance
                if (distance is not None):
                    distance = distance * 10

                print(f"Channel {channel}, VL53L1X Distance: {distance} mm")
            except RuntimeError:
                print(f"Channel {channel}, VL53L1X Error reading distance")

        

        time.sleep(1)

except KeyboardInterrupt:
    print("Program interrupted")

finally:
    bus.close()
