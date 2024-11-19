import sys
import os
import time
import board
import busio
import statistics
from smbus2 import SMBus  

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__),'AHRS_package')))
from AHRS_package import initialize_all_sensors, select_channel, read_bmx160

i2c = busio.I2C(board.SCL, board.SDA)
bus = SMBus(1) 


#Initialize
sensors_vl6180x, sensors_vl53l1x, bmp388_sensor, bmx160_sensor = initialize_all_sensors(i2c, bus)


bmp388_temperatures = []
bmp388_pressures = []
bmx160_accels = {"x": [], "y": [], "z": []}
vl6180x_distances = {}
vl53l1x_distances = {}

last_mean_calculation_time = time.time()

try:
    while True:

        current_time = time.time()

        # Read BMP388
        if bmp388_sensor:
            select_channel(i2c, 0)
            with open("data/bmp388.txt", "w") as bmp388_file:
                 
                try:
                    temperature = bmp388_sensor.temperature
                    pressure = bmp388_sensor.pressure
                    bmp388_file.write(f"BMP388 Temperature: {temperature:.2f} C, Pressure: {pressure:.2f} hPa\n\n")  
                except RuntimeError:
                    print("BMP388 Error reading data")

        # Read BMX160
        if bmx160_sensor:
            select_channel(i2c, 0)
            with open("data/bmx160.txt", "w") as bmx160_file:
                try:
                    accel_x, accel_y, accel_z = read_bmx160(bus)
                    if accel_x is not None:
                        bmx160_file.write(f"BMX160 Accel X: {accel_x}, Accel Y: {accel_y}, Accel Z: {accel_z}\n\n")  
                    else:
                        print("BMX160 Error reading data")
                except RuntimeError:
                    print("BMX160 Error reading data")

           
            # Read VL6180X
            for channel, sensor in sensors_vl6180x.items():
                select_channel(i2c, channel)
                try:
                    distance = sensor.range
                    if channel not in vl6180x_distances:
                        vl6180x_distances[channel] = []
                    vl6180x_distances[channel].append(distance)

                    
                    
                except RuntimeError:
                    print(f"Channel {channel}, VL6180X Error reading distance")

            # Read VL53L1X
            for channel, sensor in sensors_vl53l1x.items():
                select_channel(i2c, channel)
                try:
                    distance = sensor.distance
                    if (distance is not None):
                        distance = distance * 10 #Convert to mm
                    if channel not in vl53l1x_distances:
                        vl53l1x_distances[channel] = []
                    vl53l1x_distances[channel].append(distance)


                except RuntimeError:
                    print(f"Channel {channel}, VL53L1X Error reading distance")

        #Calculate and print means od distances every 0.25 second
        if current_time - last_mean_calculation_time >= 0.25:
            with open("data/distance.txt", "w") as distance_file:
             
                for channel, distances in vl6180x_distances.items():
                    distances = [d for d in distances if d is not None] #Remove none values
                    if distances:
                        mean_distance = statistics.mean(distances)

                        distance_file.write(f"Mean Vl6180X Channel {channel} Distance: {mean_distance:.2f} mm\n")

                        distances.clear()
                
                for channel, distances in vl53l1x_distances.items():
                    distances = [d for d in distances if d is not None]
                    if distances:
                        mean_distance = statistics.mean(distances)
                    
                        distance_file.write(f"Mean VL53L1X Channel {channel} Distance: {mean_distance:.2f} mm\n")

                        distances.clear()

                last_mean_calculation_time = current_time
        
except KeyboardInterrupt:
    print("Program interrupted")

finally:
    bus.close()
