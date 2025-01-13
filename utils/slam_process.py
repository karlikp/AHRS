import os
import sys

# Adding dir 'utils' to sys.path based on main project dir
sys.path.append(os.path.join(os.path.dirname(__file__), 'lib'))
from lib.Slam_2D.slam import SLAM

sys.path.append(os.path.join(os.path.dirname(__file__), 'module'))
from module.IMU_calibrator import IMU_calibrator

def slam_process(data_manager):
    
    algorithm = "icp"
    slam = SLAM(algorithm) # imu_path, lid_path)
    
    #standby IMU
    """
    stby_imu = data_manager.get_stby_imu()
    stby_quaternions = data_manager.get_stby_quaternions()
    slam.initialize_imu(stby_imu, stby_quaternions) 
    """
    
    #static calibration for IMU
    imu_calibrator = IMU_calibrator()
    imu_calibrator.calibrate_imu_once(data_manager)
    
    slam.ekf_real_time(data_manager, imu_calibrator)
    
    #slam.postprocess()

    # # Displaying results
    # print("Plotting results\n" + 50*"=")
    # slam.plot_results()
    # print("Deviation errors\n" + 50*"=")
    # print("RMSE for trajectory estimate:", slam.RMSE_traj, "m")
    # if algorithm == "icp":
    #     print("RMSE for mapping estimate:", slam.RMSE_wall_icp, "m")
    # if algorithm == "feature":
    #     print("RMSE for mapping estimate:", slam.RMSE_wall_feature, "m")
    # print("Plotting trajectory error\n" + 50*"=")
    # slam.plot_traj_error()
