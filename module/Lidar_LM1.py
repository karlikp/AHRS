import sys
sys.path.append("/home/karol/Desktop/repos/SLAM/unitree_lidar_sdk_pybind/*")
import unitree_lidar_sdk_pybind
import time
import queue
import struct
from module.Mqtt_module import Mqtt

class Lidar_LM1:

    imu_queue = queue.Queue()
    mqtt_cloud_queue = queue.Queue()
    slam_cloud_queue = queue.Queue()
    dirty_queue = queue.Queue()

    def __init__(self):
        self.is_dirty = False
        self.lidar = unitree_lidar_sdk_pybind.UnitreeLidarWrapper()
        self.lidar.set_working_mode(1)  # NORMAL
        time.sleep(1)

    def check_init(self):
        if self.lidar.initialize():
            print("Unilidar initialization succeeded.")
        else:
            print("Unilidar initialization failed! Exiting.")

    def check_dirty(self):
        count_percentage = 0
        dirty_output = []

        while True:
            lidar_dirty = self.lidar.get_dirty_percentage()
            if lidar_dirty is not None:
                #lidar_dirty: Float
                dirty_output.append(lidar_dirty)


                if count_percentage > 2:
                    break
                if lidar_dirty > 10:
                    self.is_dirty = True
                    packed_data = bytearray(struct.pack('f', lidar_dirty))
                    self.dirty_queue.put(packed_data)
                    print("The protection cover is too dirty! Please clean it right now! Exiting.")
                    exit(0)
                count_percentage += 1
            time.sleep(0.5)
        
        # Save Float[4] to queue
        for value in dirty_output:
            packed_data = bytearray(struct.pack('f', value))
            self.dirty_queue.put(packed_data)

    def parsing_data(self):
        print("\nParsing data (PointCloud and IMU)...")

        while True:
            result = self.lidar.check_message()

            if result == "IMU":
                lidar_imu = self.lidar.get_imu_data()

                if lidar_imu:
                    #Output data: 1)Timestamp: Double, 2)quaternion: Table of Float
                    mqtt_packed_data = bytearray(
                            struct.pack('d4f', lidar_imu['timestamp'], *lidar_imu['quaternion'])
                        )
                    self.imu_queue.put(mqtt_packed_data)         
                else:
                    print("No IMU data received.")

            elif result == "POINTCLOUD":
                lidar_cloud = self.lidar.get_cloud_data()

                if lidar_cloud:
                    # Output data: 1)Timestamp: Float, 2)Cloud size (amount points): Int,
                    # 3)Points {x, y, z, intensity, time}: Floats, ring: Uint32_t - 4 bytes
                    
                    points = lidar_cloud['points']
                    timestamp = lidar_cloud['timestamp']
                    
                    mqtt_packed_data = bytearray(struct.pack('fI', timestamp, len(points)))
                    slam_packed_data = []

                    for point in points:
                        x, y, z, intensity, time, ring = point
                        mqtt_point_data = struct.pack('fffffI', x, y, z, intensity, time, ring)
                        slam_point_data = [x, y, z]
                        mqtt_packed_data.extend(mqtt_point_data)  
                        slam_packed_data.extend(slam_point_data)

                    print(f"Saved to queue: {slam_packed_data}")
                    
                    self.mqtt_cloud_queue.put(mqtt_packed_data)
                else:
                    print("Lack of cloud points")


    def get_imu(self): 
        try:
            return self.imu_queue.get_nowait()
        except queue.Empty:
            print("\nEmpty queue imu_lidar")
            return None

    def get_cloud(self):
        try:
            return self.mqtt_cloud_queue.get_nowait()
        except queue.Empty:
            print("\nEmpty queue cloud")
            return None

    def get_dirty(self):

        try:
            return self.dirty_queue.get_nowait()
        except queue.Empty:
            #print("\nEmpty queue dirty")
            return None


