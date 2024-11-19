import time
import unitree_lidar_sdk_pybind
import sys

# Create Lidar wrapper object
lidar = unitree_lidar_sdk_pybind.UnitreeLidarWrapper()

if lidar.initialize():
    print("Unilidar initialization succeeded.")
else:
    print("Unilidar initialization failed! Exiting.")
    exit(-1)

print("Setting Lidar working mode to: NORMAL...")
lidar.set_working_mode(1)  # NORMAL
time.sleep(1)

# Create data directory if it doesn't exist
import os
if not os.path.exists("data"):
    os.makedirs("data")

print("\nChecking dirty percentage...")
count_percentage = 0

while True:
    dirty_percentage = lidar.get_dirty_percentage()
    if dirty_percentage is not None:  

        # Open the file in append mode
        with open("data/dirty_percentage.txt", "a") as file:
            file.write(f"Dirty Percentage = {dirty_percentage}%\n")

        if count_percentage > 2:
            break
        if dirty_percentage > 10:
            print("The protection cover is too dirty! Please clean it right now! Exiting.")
            exit(0)
        count_percentage += 1
    time.sleep(0.5)

print("\nParsing data (PointCloud and IMU)...")
while True:
    result = lidar.check_message()
    
    if result == "IMU":
        imu_data = lidar.get_imu_data()
        
        if imu_data:
            imu_output = (
                f"Timestamp: {imu_data['timestamp']}, ID: {imu_data['id']}\n"
                f"Quaternion: {imu_data['quaternion']}\n"
                f"Time delay (us): {imu_data['time_delay']}\n\n"
            )

            with open("data/imu_data.txt", "a") as imu_file:
                imu_file.write(imu_output)
                print(imu_output)
        else:
            print("No IMU data received.")

    
    elif result == "POINTCLOUD":
        cloud_data = lidar.get_cloud_data()
    
        cloud_output = (
            f"Timestamp: {cloud_data['timestamp']}, ID: {cloud_data['id']}\n"
            f"Cloud size: {len(cloud_data['points'])}, Ring Num: {cloud_data['ring_num']}\n"
            "All points:\n"
        )

        for point in cloud_data['points']:
            cloud_output += f"\t{point}\n"

        # Use append mode to add to the cloud_data.txt file
        with open("data/cloud_data.txt", "a") as cloud_file:
            cloud_file.write(cloud_output)
            print(cloud_output)
