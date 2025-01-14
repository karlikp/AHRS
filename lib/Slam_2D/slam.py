# Class file for SLAM based on IMU and LiDAR
# Accuracy Estimation of 2D SLAM Using Sensor Fusion of LiDAR and INS
# Author: Jan Xu

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from numpy.linalg import inv
# from .rotations import Quaternion, skew_symmetric
# from .feature_scan import ScanFeature, update_state, update_map
# from .icp import GlobalFrame, wraptopi
from .functions import *
import time

class SLAM:

    ###################
    ### Constructor ###
    ###################

    def __init__(self, algorithm): # imu_file, lid_file),
                #  gt_traj_file="gt_data/gt_traj.csv",
                #  gt_env_file="gt_data/gt_wall.csv"):
                
        """
        Instantiates a SLAM object based on IMU and LiDAR input data, specifying
        a LiDAR scan matching algorithm.

        Args:
            algorithm (str):
                String describing the scan matching algorithm. Choose between
                ´icp´ or ´feature´.
            imu_file (str):
                Path string to IMU dataset (.csv-file).
            lid_file (str):
                Path string to LiDAR dataset (.csv-file).
            gt_traj_file (str):
                Path string to ground truth trajectory (.csv-file).
                Default: "gt_data/gt_traj.csv"
            gt_env_file (str):
                Path string to ground truth environment (.csv-file).
                Default: "gt_data/gt_wall.csv"
        """

        self.algorithm = algorithm
        self.g = np.array([0, 0, -9.81])  # Wektor grawitacji (przyspieszenie ziemskie w m/s^2)
        self.Q_imu = np.eye(12) * 0.01  # 12x12 macierz, gdzie każdy element ma wariancję 0.01
        self.R_lid = np.eye(2) * 0.1  # Przykładowa macierz kowariancji LiDAR (2x2)
        
        self.max_iterations = 1000  # Domyślny limit iteracji
        self.current_iteration = 0
       

    ########################
    ### Instance methods ###
    ########################
    
    def termination_condition_met(self):
        """Określa, czy warunek zakończenia został spełniony."""
        self.current_iteration += 1
        return self.current_iteration >= self.max_iterations

 
 
    def ekf_real_time(self, manager_data, imu_calibrator):
        """
        Extended Kalman Filter adapted for real-time sensor fusion
        with LIDAR (providing only x, y), quaternions, and IMU (accelerometer, gyroscope, magnetometer).
        """
       
        
        # Initialize state and covariance (3D position, 3D velocity, and biases)
        p_km = np.zeros((3, 1))  # Initial position (x, y, z)
        v_km = np.zeros((3, 1))  # Initial velocity (vx, vy, vz)
        q_km = np.array([1, 0, 0, 0]).reshape(4, 1)  # Initial quaternion (no rotation)
        del_u_km = np.zeros((6, 1))  # Sensor biases (accelerometer, gyroscope)

        self.n = 0

        # State vector now includes position, velocity, quaternion, and biases
        state_dim = 3 + 3 + 4 + 6  # 3 position + 3 velocity + 4 quaternion + 6 biases
        p_cov_km = np.eye(state_dim) * 0.1  # Initialize covariance matrix

        # Initialize noise covariance matrices
        self.Q_imu = np.eye(6) * 0.01  # IMU process noise (accel and gyro)
        self.R_lid = np.eye(3) * 0.05  # LiDAR measurement noise (x, y, z)
    
        # Start real-time loop
        print("Initiating real-time Kalman filter loop...")
        prev_time = time.time()

        while True:
            # Get IMU and LiDAR data in real-time
            raw_imu_data = manager_data.get_current_imu()  # Fetch IMU data (accel, gyro, magnetometer)
            quaternions = manager_data.get_current_quaternions()  # Fetch quaternion data (orientation)
            lidar_data = manager_data.get_current_cloud()  # Fetch LIDAR data (point cloud with x, y, z)

            # Apply calibration to raw IMU data
            imu_data = imu_calibrator.apply_calibration(raw_imu_data) 
            print(imu_data)
                  
            imu_data['acc'] = np.array(imu_data['acc'])
            imu_data['gyro'] = np.array(imu_data['gyro'])
            self.n += 1
            
            current_time = time.time()
            delta_t = current_time - prev_time
            prev_time = current_time

            # IMU calibration and state propagation (3D)
            f_km = np.expand_dims(imu_data['acc'] - del_u_km[:3].flatten(), axis=1)  # 3D acceleration jako (3, 1)
            w_km = imu_data['gyro'] - del_u_km[3:].flatten()  # 3D angular velocity
            
            # Movement detection
            acc_magnitude = np.linalg.norm(imu_data['acc'] - self.g)
            gyro_magnitude = np.linalg.norm(imu_data['gyro'])
            movement_threshold_acc = 0.2  # Adjust based on noise characteristics
            movement_threshold_gyro = 0.1

            if acc_magnitude < movement_threshold_acc and gyro_magnitude < movement_threshold_gyro:
                print("No significant movement detected. Skipping state update.")
                continue
            
            # Use quaternion from sensor data instead of computing from IMU
            q_check = Quaternion(*quaternions).normalize()
            
            # Compute rotation matrix from quaternion
            C_ns = q_check.to_mat()
          
            # debug variable  
            # print("p_km shape:", p_km.shape)
            # print("v_km shape:", v_km.shape)
            # print("C_ns shape:", C_ns.shape)
            # print("f_km shape:", f_km.shape)
            # print("self.g shape:", self.g.reshape(3, 1).shape)

            # Propagate state based on IMU data (3D position and velocity)
            p_check = p_km + delta_t * v_km + (delta_t ** 2 / 2) * (C_ns.dot(f_km) - self.g.reshape(3, 1))  # 3D position
            v_check = v_km + delta_t * (C_ns.dot(f_km) - self.g.reshape(3, 1))  # 3D velocity
            del_u_check = del_u_km

            # Linearize motion model
            F, L = SLAM.state_space_model(f_km, C_ns, delta_t)

            # Ensure F has the correct size
            if F.shape != (state_dim, state_dim):
                raise ValueError(f"Shape mismatch: F should be {state_dim}x{state_dim}, but got {F.shape}")
            
            # Propagate uncertainty (3D state)
            p_cov_check = F.dot(p_cov_km).dot(F.T) + L.dot(self.Q_imu).dot(L.T)

            # LIDAR measurement update - Only x, y, z
            if lidar_data is not None and len(lidar_data) > 0:
                y_k = self.process_lidar_data(lidar_data)  # Extract position (x, y, z) from point cloud
                #print("y_k shape after process_lidar_data:", y_k.shape)  # Powinno być (3, 1)
                
                # Measurement matrix (maps state to measurement space)
                H = np.zeros((3, state_dim))
                H[:3, :3] = np.eye(3)  # Only position is observed
                
                new_state = SLAM.measurement_update(
                    y_k, p_check, v_check, q_check, del_u_check, p_cov_check, self.R_lid
                )
                p_check, v_check, q_check, del_u_check, p_cov_check = new_state
                
                # Compute Kalman gain
                S = H.dot(p_cov_check).dot(H.T) + self.R_lid
                
                # Verify residual covariance for anomaly detection
                if np.any(np.diag(S) < 1e-6):  # Threshold to avoid singularity
                    print("Skipping update due to small residual covariance")
                    continue
                
                K = p_cov_check.dot(H.T).dot(np.linalg.inv(S))
                
                 # Update state and covariance
                residual = y_k.reshape(3, 1) - H.dot(np.vstack((p_check, v_check, q_check.to_numpy().reshape(-1, 1), del_u_check)))
                state_update = K.dot(residual)
                
                # Apply update
                p_check += state_update[:3]
                v_check += state_update[3:6]
                q_check = Quaternion(*q_check.to_numpy() + state_update[6:10].flatten()).normalize()
                del_u_check += state_update[10:]
                
                # Update covariance
                p_cov_check = (np.eye(state_dim) - K.dot(H)).dot(p_cov_check)

            # Store updated state
            p_km, v_km, q_km, del_u_km, p_cov_km = p_check, v_check, q_check, del_u_check, p_cov_check

            # Print real-time position (3D)
            print(f"Real-time Position: x = {p_km[0, 0]:.2f}, y = {p_km[1, 0]:.2f}, z = {p_km[2, 0]:.2f}")

            # Termination condition (optional)
            if self.termination_condition_met():
                break


    def process_lidar_data(self, lidar_data):
        """
        Process LiDAR data to extract x, y, z position as a 3x1 numpy array.
        """
        if len(lidar_data) == 0:
            raise ValueError("LiDAR data is empty")
    
        # Assume lidar_data is a list of tuples [(x, y, z)]
        x, y, z = lidar_data[0]  # Take the first point
        return np.array([[x], [y], [z]])  # Ensure (3, 1) shape


    #####################
    ### Class methods ###
    #####################


    def measurement_update(y_k, p_check, v_check, q_check, del_u_check, p_cov_check, R_lid):
        """
        Performs the measurement update step of the Extended Kalman Filter for sensor fusion.
        
        Args:
            y_k [3x1 Numpy array]: Measurement from the sensor (e.g., LIDAR for position x, y, z).
            p_check [3x1 Numpy array]: Predicted position.
            v_check [3x1 Numpy array]: Predicted velocity.
            q_check [4x1 Numpy array]: Predicted quaternion representing orientation.
            del_u_check [6x1 Numpy array]: Predicted sensor biases.
            p_cov_check [16x16 Numpy array]: Predicted covariance matrix of the full state.
            R_lid [3x3 Numpy array]: Sensor noise covariance for LIDAR.
        
        Returns:
            Updated position, velocity, quaternion, covariance, and biases.
        """
        state_dim = 16
        measurement_dim = 3  # Assuming LIDAR provides x, y, z measurements.

        # Measurement model Jacobian (H maps state to measurement)
        H = np.zeros((measurement_dim, state_dim))
        H[:3, :3] = np.eye(3)  # Measurement is directly related to position.

        # Measurement residual (innovation)
        y_k_pred = p_check  # Predicted position from the state.
        y_residual = y_k - y_k_pred  # Residual between measurement and prediction.
           
        # Compute Kalman gain
        S = H.dot(p_cov_check).dot(H.T) + R_lid
        K_k = p_cov_check.dot(H.T).dot(np.linalg.inv(S))

        # Update state estimate
        state_update = K_k.dot(y_residual)
        
        #debug
        # print("state_update shape:", state_update.shape)  # Powinno być (16, 1)
        # print("K_k shape:", K_k.shape)  # Powinno być (16, 3)
        # print("y_residual shape:", y_residual.shape)  # Powinno być (3, 1)


        # Extract updated states
        p_check_new = p_check + state_update[:3]
        v_check_new = v_check + state_update[3:6]
        q_check_new = Quaternion(*( q_check.to_numpy() + state_update[6:10].flatten())).normalize()
        #q_check_new = Quaternion(*q_check.to_numpy() + state_update[6:10].flatten()).normalize()
        del_u_check_new = del_u_check + state_update[10:]

        # Update covariance
        p_cov_check_new = (np.eye(state_dim) - K_k.dot(H)).dot(p_cov_check)

        return p_check_new, v_check_new, q_check_new, del_u_check_new, p_cov_check_new

    
    def state_space_model(imu_f, C_ns, delta_t):
        """
        Generate state transition matrix F and noise gain matrix L for extended Kalman filter (EKF)
        with 16-dimensional state vector (position, velocity, quaternion, sensor biases).

        Args:
            imu_f [3x1 Numpy array]: Specific force vector from inertial accelerometer in [m/s^2].
            C_ns [3x3 Numpy array]: Direction cosine matrix resolving orientation to the navigation frame.
            delta_t (float): Time increment in seconds.

        Returns:
            F [16x16 Numpy array]: State transition matrix of the current time step.
            L [16x6 Numpy array]: Noise gain matrix of the current time step.
        """
        state_dim = 16
        process_noise_dim = 6  # Acceleration and angular velocity noise

        # Initialize F and L
        F = np.eye(state_dim)
        L = np.zeros((state_dim, process_noise_dim))

        # Position and velocity updates
        F[:3, 3:6] = np.eye(3) * delta_t  # Position depends on velocity
        F[3:6, 6:10] = delta_t * compute_quaternion_jacobian(C_ns, imu_f)  # Quaternion dependency

        # Orientation (quaternion) update is linearized (identity for now)
        F[6:10, 6:10] = np.eye(4)

        # Bias updates (assume no change for simplicity)
        F[10:, 10:] = np.eye(6)

        # Noise gain matrix
        L[3:6, :3] = C_ns * delta_t  # Velocity sensitivity to accelerometer noise
        L[6:10, 3:] = np.eye(4)[:, :3] * delta_t  # Orientation sensitivity to gyro noise
        L[10:, :] = np.eye(6)  # Bias sensitivity to process noise

        return F, L
    
    