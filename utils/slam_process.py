import os
import sys

# Adding dir 'utils' to sys.path based on main project dir
sys.path.append(os.path.join(os.path.dirname(__file__), 'lib'))
from lib.Slam_2D.slam import *

sys.path.append(os.path.join(os.path.dirname(__file__), 'module'))
from module.IMU_calibrator import IMU_calibrator

def slam_process(data_manager, semaphore):
    
    #static calibration for IMU
    imu_calibrator = IMU_calibrator()
    imu_calibrator.calibrate_imu_once(data_manager)
    
    calibrated_matrix = data_manager.get_lidar_calibrate()
    
    ekf_real_time(data_manager, imu_calibrator, semaphore, calibrated_matrix)
    