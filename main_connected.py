import sys
import os
import time
import board
import busio
from smbus2 import SMBus  
import threading
import unitree_lidar_sdk_pybind

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), 'AHRS_package')))
from AHRS_package import *

i2c = busio.I2C(board.SCL, board.SDA)
bus = SMBus(1)

#sensors_vl6180x, sensors_vl53l1x, bmx160_sensor = sensors_init.initialize_all_sensors(i2c, bus)

bmp388_temperatures = []
bmp388_pressures = []
bmx160_accels = {"x": [], "y": [], "z": []}
vl6180x_distances = {}
vl53l1x_distances = {}

last_mean_calculation_time = time.time()

def sensor_reading():
    global last_mean_calculation_time
    
    bmp388 = Bmp388(i2c)
    bmx160 = Bmx160(1)
    vl6180x = Vl6180x(i2c)
    vl53l1x = Vl53l1x(i2c)

    try:
        while True:
            current_time = time.time()

        
            bmp388.save_to_file()
            bmx160.save_to_file()

            vl6180x.collect_data()
            vl53l1x.collect_data()

            # for channel, sensor in sensors_vl6180x.items():
            #     select_channel(i2c, channel)
            #     try:
            #         distance = sensor.range
            #         if channel not in vl6180x_distances:
            #             vl6180x_distances[channel] = []
            #         vl6180x_distances[channel].append(distance)
            #     except RuntimeError:
            #         print(f"Channel {channel}, VL6180X Error reading distance")

            # for channel, sensor in sensors_vl53l1x.items():
            #     select_channel(i2c, channel)
            #     try:
            #         distance = sensor.distance
            #         if distance is not None:
            #             distance = distance * 10  # Convert to mm
            #         if channel not in vl53l1x_distances:
            #             vl53l1x_distances[channel] = []
            #         vl53l1x_distances[channel].append(distance)
            #     except RuntimeError:
            #         print(f"Channel {channel}, VL53L1X Error reading distance")



            if current_time - last_mean_calculation_time >= 0.25:
            
                vl6180x.save_to_file()
                vl53l1x.save_to_file()

                # with open("data/distance.txt", "w") as distance_file:
                #     for channel, distances in vl6180x_distances.items():
                #         distances = [d for d in distances if d is not None]  # Remove None values
                #         if distances:
                #             mean_distance = statistics.mean(distances)
                #             distance_file.write(f"Mean Vl6180X Channel {channel} Distance: {mean_distance:.2f} mm\n")
                #             distances.clear()

                    # for channel, distances in vl53l1x_distances.items():
                    #     distances = [d for d in distances if d is not None]
                    #     if distances:
                    #         mean_distance = statistics.mean(distances)
                    #         distance_file.write(f"Mean VL53L1X Channel {channel} Distance: {mean_distance:.2f} mm\n")
                    #         distances.clear()

                last_mean_calculation_time = current_time
    except KeyboardInterrupt:
        print("Program interrupted")


# def lidar_reading():
#     lidar = unitree_lidar_sdk_pybind.UnitreeLidarWrapper()

#     if lidar.initialize():
#         print("Unilidar initialization succeeded.")
#     else:
#         print("Unilidar initialization failed! Exiting.")
#         return None

#     print("Setting Lidar working mode to: NORMAL...")
#     lidar.set_working_mode(1)  # NORMAL
#     time.sleep(1)

#     open("data/dirty_percentage.txt", "w").close()
#     open("data/imu_data.txt", "w").close()
#     open("data/cloud_data.txt", "w").close()

#     print("\nChecking dirty percentage...")
#     count_percentage = 0

#     while True:
#         dirty_percentage = lidar.get_dirty_percentage()
#         if dirty_percentage is not None:
#             # Open the file in append mode
#             with open("data/dirty_percentage.txt", "a") as file:
#                 file.write(f"Dirty Percentage = {dirty_percentage}%\n")

#             if count_percentage > 2:
#                 break
#             if dirty_percentage > 10:
#                 print("The protection cover is too dirty! Please clean it right now! Exiting.")
#                 exit(0)
#             count_percentage += 1
#         time.sleep(0.5)

#     print("\nParsing data (PointCloud and IMU)...")
#     while True:
#         result = lidar.check_message()

#         if result == "IMU":
#             imu_data = lidar.get_imu_data()

#             if imu_data:
#                 imu_output = (
#                     f"Timestamp: {imu_data['timestamp']}, ID: {imu_data['id']}\n"
#                     f"Quaternion: {imu_data['quaternion']}\n"
#                     f"Time delay (us): {imu_data['time_delay']}\n\n"
#                 )

#                 with open("data/imu_data.txt", "w") as imu_file:
#                     imu_file.write(imu_output)
#             else:
#                 print("No IMU data received.")

#         elif result == "POINTCLOUD":
#             cloud_data = lidar.get_cloud_data()

#             cloud_output = (
#                 f"Timestamp: {cloud_data['timestamp']}, ID: {cloud_data['id']}\n"
#                 f"Cloud size: {len(cloud_data['points'])}, Ring Num: {cloud_data['ring_num']}\n"
#                 "All points:\n"
#             )

#             for point in cloud_data['points']:
#                 cloud_output += f"\t{point}\n"

#             with open("data/cloud_data.txt", "w") as cloud_file:
#                 cloud_file.write(cloud_output)


if __name__ == "__main__":
    
    sensor_thread = threading.Thread(target=sensor_reading)
    #lidar_thread = threading.Thread(target=lidar_reading)

    sensor_thread.start()
    #lidar_thread.start()

    sensor_thread.join()
    #lidar_thread.join()
