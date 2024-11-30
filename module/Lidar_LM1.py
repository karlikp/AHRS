import sys
sys.path.append("/home/karol/Desktop/repos/SLAM/unitree_lidar_sdk_pybind/*")
import unitree_lidar_sdk_pybind
import time

class Lidar_LM1:

    

    def __init__(self):
        self.is_dirty = False
        self.lidar = unitree_lidar_sdk_pybind.UnitreeLidarWrapper()
        print("Setting Lidar working mode to: NORMAL...")
        self.lidar.set_working_mode(1)  # NORMAL
        self.open_file()
        time.sleep(1)

    def check_init(self):
        if self.lidar.initialize():
            print("Unilidar initialization succeeded.")
           
        else:
            print("Unilidar initialization failed! Exiting.")

    def check_dirty(self):
        count_percentage = 0
        dirty_output = (
            f"{{"
            f"\n\tDirty percentage: [\n"         
        )
        while True:
            lidar_dirty = self.lidar.get_dirty_percentage()
            if lidar_dirty is not None:

                dirty_output += f"\t\t{{{lidar_dirty}}},\n"

                if count_percentage > 2:
                    break
                if lidar_dirty > 10:
                    self.is_dirty = True
                    print("The protection cover is too dirty! Please clean it right now! Exiting.")
                    exit(0)
                count_percentage += 1
            time.sleep(0.5)
        
        dirty_output += f"\t]\n}}\n"

        with open("/home/karol/Desktop/repos/SLAM/data/package/lidar_dirty.txt", "a") as file:
                    file.write(f"Dirty Percentage = {lidar_dirty}%\n")

    def open_file(self):
        open("/home/karol/Desktop/repos/SLAM/data/current/lidar_imu.txt", "w").close()
        open("/home/karol/Desktop/repos/SLAM/data/current/lidar_cloud.txt", "w").close()

        open("/home/karol/Desktop/repos/SLAM/data/package/lidar_dirty.txt", "w").close()
        open("/home/karol/Desktop/repos/SLAM/data/package/lidar_imu.txt", "w").close()
        open("/home/karol/Desktop/repos/SLAM/data/package/lidar_cloud.txt", "w").close()

    def parsing_data(self):
        print("\nParsing data (PointCloud and IMU)...")

        while True:
            result = self.lidar.check_message()

            if result == "IMU":
                lidar_imu = self.lidar.get_imu_data()

                if lidar_imu:
                    imu_output = (
                        f"Timestamp: {lidar_imu['timestamp']}, ID: {lidar_imu['id']}\n"
                        f"Quaternion: {lidar_imu['quaternion']}\n"
                        f"Time delay (us): {lidar_imu['time_delay']}\n\n"
                    )

                    file_path1 = "/home/karol/Desktop/repos/SLAM/data/current/lidar_imu.txt"
                    file_path2 = "/home/karol/Desktop/repos/SLAM/data/package/lidar_imu.txt"

                    with open(file_path1, "w") as imu_file1, open(file_path2, "a") as imu_file2:
                        imu_file1.write(imu_output)
                        imu_file2.write(imu_output)
                else:
                    print("No IMU data received.")

            elif result == "POINTCLOUD":
                lidar_cloud = self.lidar.get_cloud_data()

                cloud_output = (
                    f"{{"
                    f"\n\tID: {lidar_cloud['id']},"
                    f"\n\tTimestamp: {lidar_cloud['timestamp']},"
                    f"\n\tCloud size: {len(lidar_cloud['points'])},"
                    f"\n\tRing Num: {lidar_cloud['ring_num']},"
                    "\n\tAll points: [\n"
                )

                for point in lidar_cloud['points']:
                    cloud_output += f"\t\t{point}\n"
                
                cloud_output += (
                    f"\t]\n}},\n"
                )

                file_path1 = "/home/karol/Desktop/repos/SLAM/data/current/lidar_cloud.txt"
                file_path2 = "/home/karol/Desktop/repos/SLAM/data/package/lidar_cloud.txt"

                with open(file_path1, "w") as cloud_file1, open(file_path2, "a") as cloud_file2:
                    cloud_file1.write(cloud_output)
                    cloud_file2.write(cloud_output)

    def get_is_dirty(self):
        return self.is_dirty

