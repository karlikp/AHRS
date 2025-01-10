import sys
import os
# Adding dir 'unitree_lidar_sdk_pybind' to sys.path based on main project dir
sys.path.append(os.path.join(os.path.dirname(__file__), 'unitree_lidar_sdk_pybind'))

import unitree_lidar_sdk_pybind
import time
import queue
import struct

class Lidar_LM1:

   
    
    

    def __init__(self):
        self.mqtt_imu_queue = queue.Queue()
        self.mqtt_cloud_queue = queue.Queue()
        self.mqtt_dirty_queue = queue.Queue()
        
        self.current_quaternions = []
        self.current_cloud = []
        self.stby_quaternions = []
        
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
                print(f"Current lidar dirty percentage: {lidar_dirty}%")  # Dodaj logowanie
                #lidar_dirty: Float
                dirty_output.append(lidar_dirty)


                if count_percentage > 2:
                    break
                if lidar_dirty > 10:
                    self.is_dirty = True
                    packed_data = bytearray(struct.pack('f', lidar_dirty))
                    self.mqtt_dirty_queue.put(packed_data)
                    print("The protection cover is too dirty! Please clean it right now! Exiting.")
                    exit(0)
                count_percentage += 1
            time.sleep(0.5)
        
        # Save Float[4] to queue
        for value in dirty_output:
            packed_data = bytearray(struct.pack('f', value))
            self.mqtt_dirty_queue.put(packed_data)

    def parsing_data(self):
        print("\nParsing data (PointCloud and IMU)...")

        while True:
            result = self.lidar.check_message()

            if result == "IMU":
                lidar_imu = self.lidar.get_imu_data()

                if lidar_imu:
   
                    #Output data: 1)Timestamp: Double, 2)quaternion: Table of Float
                    packed_data = bytearray(
                            struct.pack('d4f', lidar_imu['timestamp'], *lidar_imu['quaternion'])
                        )
                    self.mqtt_imu_queue.put(packed_data) 
                    
                    # Dodawanie tylko quaternionów do listy standby, dopóki nie osiągnie 700
                    if len(self.stby_quaternions) < 700:
                        self.stby_quaternions.append(lidar_imu['quaternion'])
                    else:
                        self.current_quaternions[:] = [*lidar_imu['quaternion']]
                      
                         
                else:
                    print("No IMU data received.")

            elif result == "POINTCLOUD":
                lidar_cloud = self.lidar.get_cloud_data()

                if lidar_cloud:
                    # Output data: 1)Timestamp: Float, 2)Cloud size (amount points): Int,
                    # 3)Points {x, y, z, intensity, time}: Float, ring: Uint32_t - 4 bytes
                    
                    points = lidar_cloud['points']
                    timestamp = lidar_cloud['timestamp']
                    
                    packed_data = bytearray(struct.pack('fI', timestamp, len(points)))
                    temp_cloud = []

                    for point in points:
                        x, y, z, intensity, time, ring = point
                        point_data = struct.pack('fffffI', x, y, z, intensity, time, ring)
                        packed_data.extend(point_data) 
                        temp_cloud.append((x,y))
                        
                    #print(f"{x},{y},{z},{intensity}, {time}, {ring}")
                        
                    self.current_cloud[:] = temp_cloud # 2D 
                    self.mqtt_cloud_queue.put(packed_data)
                    
                else:
                    print("Lack of cloud points")

    def get_dirty(self):
        return self.is_dirty
    
    def get_current_quaternions(self):
        return self.current_quaternions
    
    def get_current_cloud(self):
        return self.current_cloud
    def get_stby_quaternions(self):
        return self.stby_quaternions
    