import sys
import os
import threading
import numpy as np
# Adding dir 'unitree_lidar_sdk_pybind' to sys.path based on main project dir
sys.path.append(os.path.join(os.path.dirname(__file__), 'lib','unitree_lidar_sdk_pybind'))

import unitree_lidar_sdk_pybind
import time
import queue
import struct

from .Compass import Compass
from scipy.spatial.transform import Rotation as R

class Lidar_LM1:

    def __init__(self):
        self.mqtt_imu_queue = queue.Queue()
        self.mqtt_cloud_queue = queue.Queue()
        self.mqtt_dirty_queue = queue.Queue()
        self.mqtt_trans_matrix_queue = queue.Queue()  #TO DO
        
        self.azimuth = None
        self.calibration_finish = False
        self.compass_is_calibrated = False
        self.compass = Compass()
        self.current_quaternions = []
        self.current_cloud = []
        self.stby_quaternions = []
        
        self.is_dirty = False
        self.lidar = unitree_lidar_sdk_pybind.UnitreeLidarWrapper()
        self.lidar.set_working_mode(1)  # NORMAL
        
        self.collected_matrices = []
        self.tf_matrix = None
        self.matrix_mutex = threading.Lock()
        self.calibre_ready_event = threading.Event()
        
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
            lidar_dirty = self.lidar.get_dirty_percentage() # Error with get_dirty_percentage!!!
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
        print(f"Parsing data (PointCloud and IMU)...")
        
        

        while True:
            result = self.lidar.check_message()

            if result == "IMU":
                
                lidar_imu = self.lidar.get_imu_data()

                if lidar_imu:
   
                    
                    quaternions_pack = bytearray(
                            struct.pack('d4f', lidar_imu['timestamp'], *lidar_imu['quaternion'])
                        )
                    
                    if self.calibration_finish:
                        self.mqtt_imu_queue.put(quaternions_pack) 
                    
                    if not self.compass_is_calibrated:
                        self.compass.calibrate(lidar_imu['quaternion'])
                        self.compass_is_calibrated = True
                    
                    # Get compass data and sent to mqtt
                    self.azimuth = self.compass.get_corrected_azimuth(lidar_imu['quaternion'])
                               
                else:
                    print("No IMU data received.")

            elif result == "POINTCLOUD":
                
                lidar_cloud = self.lidar.get_cloud_data()

                if lidar_cloud:
                    
                    points = lidar_cloud['points']
                    timestamp = lidar_cloud['timestamp']
                    
                    quaternions_pack = bytearray(struct.pack('fI', timestamp, len(points)))
                                    
                    temp_cloud = []

                    for point in points:
                        x, y, z, intensity, point_time, ring = point
                        point_data = struct.pack('fffffI', x, y, z, intensity, point_time, ring)
                        quaternions_pack.extend(point_data) 
                        temp_cloud.append((x,y,z))
                    
                    if self.calibration_finish:
                        self.mqtt_cloud_queue.put(quaternions_pack)
                    
                    if 'icp' in lidar_cloud and np.any(self.tf_matrix != lidar_cloud['icp']):
                        # Store the first 0 transformation matrices
                        if len(self.collected_matrices) < 10:
                            self.collected_matrices.append(lidar_cloud['icp'])
                            
                        
                        else:
                            self.calibration_finish = True
                            self.calibre_ready_event.set()
                            self.tf_matrix = lidar_cloud['icp']
                            self.matrix_timestamp = lidar_cloud['matrix_timestamp']
                                                 
                else:
                    print("Lack of cloud points")
                    
    def lidar_calibrate(self):
        if not self.collected_matrices:
            raise ValueError("Lack of calibrate data")

       
        if not all(isinstance(mat, np.ndarray) and mat.shape == (4, 4) for mat in self.collected_matrices):
            raise ValueError("Invalid matrix format in collected_matrices. Expected a list of 4x4 matrices.")

        collected_matrices_np = np.array(self.collected_matrices)

        avg_rotation = np.mean(collected_matrices_np[:, :3, :3], axis=0)
        avg_translation = np.mean(collected_matrices_np[:, :3, 3], axis=0)

        U, _, Vt = np.linalg.svd(avg_rotation)
        corrected_rotation = U @ Vt

        calibrated_matrix = np.eye(4)
        calibrated_matrix[:3, :3] = corrected_rotation
        calibrated_matrix[:3, 3] = avg_translation

        print(f"Received calibrated_matrix: {calibrated_matrix}")
        print(f"Shape of calibrated_matrix: {calibrated_matrix.shape}")
    
        return calibrated_matrix
    
    def get_dirty(self):
        return self.is_dirty
    
    def get_transformation_matrix(self):
        return self.tf_matrix
    
    