import unitree_lidar_sdk_pybind
import time

class Lidar_LM1:

    def __init__(self):
        self.lidar = unitree_lidar_sdk_pybind.UnitreeLidarWrapper()
        print("Setting Lidar working mode to: NORMAL...")
        self.lidar.set_working_mode(1)  # NORMAL
        self.open_file()
        time.sleep(1)

        self.ready_to_work = False

    def check_init(self):
        if self.lidar.initialize():
            print("Unilidar initialization succeeded.")
            self.ready_to_work = True
        else:
            print("Unilidar initialization failed! Exiting.")

    def check_dirty(self):
        count_percentage = 0
        while True:
            dirty_percentage = self.lidar.get_dirty_percentage()
            if dirty_percentage is not None:
                # Open the file in append mode
                with open("/home/karol/Desktop/repos/SLAM/data/package/dirty_percentage.txt", "a") as file:
                    file.write(f"Dirty Percentage = {dirty_percentage}%\n")

                if count_percentage > 2:
                    break
                if dirty_percentage > 10:
                    print("The protection cover is too dirty! Please clean it right now! Exiting.")
                    exit(0)
                count_percentage += 1
            time.sleep(0.5)

    def open_file(self):
        open("/home/karol/Desktop/repos/SLAM/data/current/imu_data.txt", "w").close()
        open("/home/karol/Desktop/repos/SLAM/data/current/cloud_data.txt", "w").close()

        open("/home/karol/Desktop/repos/SLAM/data/package/dirty_percentage.txt", "w").close()
        open("/home/karol/Desktop/repos/SLAM/data/package/imu_data.txt", "w").close()
        open("/home/karol/Desktop/repos/SLAM/data/package/cloud_data.txt", "w").close()

    def parsing_data(self):
        print("\nParsing data (PointCloud and IMU)...")

        while True:
            result = self.lidar.check_message()

            if result == "IMU":
                imu_data = self.lidar.get_imu_data()

                if imu_data:
                    imu_output = (
                        f"Timestamp: {imu_data['timestamp']}, ID: {imu_data['id']}\n"
                        f"Quaternion: {imu_data['quaternion']}\n"
                        f"Time delay (us): {imu_data['time_delay']}\n\n"
                    )

                    file_path1 = "/home/karol/Desktop/repos/SLAM/data/current/imu_data.txt"
                    file_path2 = "/home/karol/Desktop/repos/SLAM/data/current/imu_data_additional.txt"

                    with open(file_path1, "w") as imu_file1, open(file_path2, "a") as imu_file2:
                        imu_file1.write(imu_output)
                        imu_file2.write(imu_output)
                else:
                    print("No IMU data received.")

            elif result == "POINTCLOUD":
                cloud_data = self.lidar.get_cloud_data()

                cloud_output = (
                    f"Timestamp: {cloud_data['timestamp']}, ID: {cloud_data['id']}\n"
                    f"Cloud size: {len(cloud_data['points'])}, Ring Num: {cloud_data['ring_num']}\n"
                    "All points:\n"
                )

                for point in cloud_data['points']:
                    cloud_output += f"\t{point}\n"

                with open("/home/karol/Desktop/repos/SLAM/data/current/cloud_data.txt", "w") as cloud_file:
                    cloud_file.write(cloud_output)

    def get_ready_to_work(self):
        return self.ready_to_work

