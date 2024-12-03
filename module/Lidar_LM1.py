import sys
sys.path.append("/home/karol/Desktop/repos/SLAM/unitree_lidar_sdk_pybind/*")
import unitree_lidar_sdk_pybind
import time
import queue
from Mqtt import Mqtt

class Lidar_LM1:

    imu_queue = queue.Queue()
    cloud_queue = queue.Queue()
    dirty_queue = queue.Queue()

    def __init__(self):
        self.is_dirty = False
        self.lidar = unitree_lidar_sdk_pybind.UnitreeLidarWrapper()
        self.lidar.set_working_mode(1)  # NORMAL

        
        time.sleep(1)

    def __init__(self, topic):
        self.topic = topic

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
                    self.dirty_queue.put(lidar_dirty)
                    print("The protection cover is too dirty! Please clean it right now! Exiting.")
                    exit(0)
                count_percentage += 1
            time.sleep(0.5)
        
        # Save Float[4] to queue
        self.dirty_queue.put(dirty_output)

    def parsing_data(self):
        print("\nParsing data (PointCloud and IMU)...")

        while True:
            result = self.lidar.check_message()

            if result == "IMU":
                lidar_imu = self.lidar.get_imu_data()
                
                #Output data: 1)Timestamp: Double, 2)quaternion: Table of Float
                if lidar_imu:
                    imu_output = [ lidar_imu['timestamp'],lidar_imu['quaternion'] ]
                    
                    #Save data to queue
                    self.imu_queue.put(imu_output)

                else:
                   print("No IMU data received.")

            elif result == "POINTCLOUD":
                lidar_cloud = self.lidar.get_cloud_data()

                #Output data: 1)Timestamp: Float, 2)Cloud size(amout points): Int, 
                #3)Points {x,y,z,intensity,time}:Float, ring: Uint32_t - 4bity
                cloud_output = [
                    lidar_cloud['timestamp'], 
                    len(lidar_cloud['points']),
                    lidar_cloud['points']
                    ]
                
                #Save data to queue
                self.cloud_queue.put(cloud_output)  


    #Get imu data if there are available
    def get_imu(self):
            
        try:
            return self.imu_queue.get_nowait()
        except queue.Empty:
            return None

    #Get cloud data if there are available 
    def get_cloud(self):

        try:
            return self.cloud_queue.get_nowait()
        except queue.Empty:
            return None

    #Get dirty data if there are available
    def get_dirty(self):

        try:
            return self.dirty_queue.get_nowait()
        except queue.Empty:
            return None


