# Class file for SLAM based on IMU and LiDAR
# Accuracy Estimation of 2D SLAM Using Sensor Fusion of LiDAR and INS
# Author: Jan Xu

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from numpy.linalg import inv
from .rotations import Quaternion, skew_symmetric
from .feature_scan import ScanFeature, update_state, update_map
from .icp import GlobalFrame, wraptopi
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
        # self.imu_file = imu_file
        # self.lid_file = lid_file
        # self.gt_traj = pd.read_csv(gt_traj_file, header=None, usecols=range(2)).values
        # self.gt_wall = pd.read_csv(gt_env_file, header=None, usecols=range(2)).values


    ########################
    ### Instance methods ###
    ########################
    
    def termination_condition_met(self):
        """Określa, czy warunek zakończenia został spełniony."""
        self.current_iteration += 1
        return self.current_iteration >= self.max_iterations

    def get_imu_data_realtime(self, imu_data_stream, stby=700, offset_quat=True, gravity_bias_est=False):
        """
        Reads, cleans, and assigns IMU data to object attributes in real-time.

        Args:
            imu_data_stream (list):
                A real-time list containing IMU data with the following format:
                [
                    {"timestamp": <float>, 
                    "q_x": <float>, "q_y": <float>, "q_z": <float>, "q_w": <float>,
                    "om_x": <float>, "om_y": <float>, "om_z": <float>,
                    "a_x": <float>, "a_y": <float>, "a_z": <float>
                    },
                    ...
                ]
            stby (int):
                Number of initial IMU readings (standby readings) before first movement. Default: 700.
            offset_quat (bool):
                If True, the quaternion readings will be offset such that
                the initial heading is zero. Default: True.
            gravity_bias_est (bool):
                If True, the initial bias estimation, including estimating
                gravity component in the z-direction, will be done by averaging
                across the first ´stby´ number of IMU readings. Default: False.
        """

        # Convert the stream of dictionaries into structured numpy arrays
        imu = pd.DataFrame(imu_data_stream)  # Convert list of dicts to DataFrame for easier manipulation

        # Ensure the required columns are present
        required_columns = ["timestamp", "q_x", "q_y", "q_z", "q_w", 
                            "om_x", "om_y", "om_z", "a_x", "a_y", "a_z"]
        if not all(col in imu.columns for col in required_columns):
            raise ValueError(f"IMU data is missing required fields. Expected fields: {required_columns}")

        # Assign arrays for timestamp, linear acceleration, and rotational velocity
        self.imu_t = np.round(imu["timestamp"].values / 1e9, 3)  # converting to seconds with three decimal places
        self.imu_f = imu[["a_x", "a_y", "a_z"]].values
        self.imu_w = imu[["om_x", "om_y", "om_z"]].values
        self.imu_q = imu[["q_w", "q_x", "q_y", "q_z"]].values
        self.n = self.imu_f.shape[0]  # number of IMU readings

        # Offset quaternion readings to ensure initial heading is zero
        if offset_quat:
            # Transform quaternions to Euler angles
            phi = np.ndarray((self.n, 3))
            for i in range(self.n):
                phi[i, :] = Quaternion(*self.imu_q[i, :]).normalize().to_euler()

            # Shift yaw angle such that the initial yaw equals zero
            inityaw = np.mean(phi[:stby, 2])  # Estimated initial yaw
            tf_yaw = wraptopi(phi[:, 2] - inityaw)  # Transformed yaw

            # Transform Euler angles back to quaternions
            phi[:, 2] = tf_yaw
            for i in range(self.n):
                self.imu_q[i, :] = Quaternion(euler=phi[i, :]).normalize().to_numpy()

        # Estimate gravity bias if required
        if gravity_bias_est:
            imu_f_trans = np.ndarray((stby, 3))
            for i in range(stby):
                C_ns = Quaternion(*self.imu_q[i, :]).normalize().to_mat()
                imu_f_trans[i] = C_ns.dot(self.imu_f[i, :])

            self.g = np.mean(imu_f_trans, axis=0)  # bias + gravity
        else:
            self.g = np.array([0, 0, 9.80665])  # gravity


    def get_lidar_data(self):
        """
        Loads and assigns LiDAR data to object attributes.

        """

        lid = pd.read_csv(self.lid_file, usecols=[2] + list(range(11, 693)))

        # Assign arrays for timestamp and range data
        self.lid_t = np.round(lid["field.header.stamp"].values / 1e9, 3) # converting to seconds with three decimal places
        self.lid_r = lid.iloc[:,1:].values # WK


    def plot_ground_truth(self):
        """
        Plots a visualization of the ground truth trajectory and environment.

        """

        fig = plt.figure(figsize=(9,7))
        plt.plot(self.gt_traj[:,0], self.gt_traj[:,1], 'r.-', markersize=1, label="Trajectory")
        plt.plot(self.gt_traj[0,0], self.gt_traj[0,1], 'ro', markersize=10, label="Trajectory start")
        plt.plot(self.gt_traj[-1,0], self.gt_traj[-1,1], 'rx', markersize=12, label="Trajectory end")
        plt.plot(self.gt_wall[:2,0], self.gt_wall[:2,1], 'k-', linewidth=1, label="Maze walls")
        plt.plot(self.gt_wall[:,0], self.gt_wall[:,1], 'k.', markersize=0.5)
        plt.xlim(-0.1, 1.7)
        plt.ylim(-0.1, 1.3)
        plt.legend(loc="right")
        plt.grid(alpha=0.5)
        plt.title("Ground truth of experiment")
        plt.show()


    def set_params(self, imu_f=0.05, imu_w=10*np.pi/180,
                   imu_bias_f=0.0005, imu_bias_w=0.01*np.pi/180,
                   lid_p=0.01, lid_r=0.5*np.pi/180):
        """
        Sets standard deviation parameters and constructs sensor covariance
        matrices Q_imu (process noise covariance) and R_lid (measurement noise
        covariance).

        Args:
            imu_f (float):
                Standard deviation associated with uncertainty of inertial
                accelerometer sensor in [m/s^2]. Default: 0.05 m/s^2.
            imu_w (float):
                Standard deviation associated with uncertainty of inertial
                gyroscope sensor in [rad/s]. Default: 1.745e-1 rad/s ≈ 10°/s.
            imu_bias_f (float):
                Standard deviation associated with uncertainty of the bias in
                inertial accelerometer sensor in [m/s^2]. Default: 0.0005 m/s^2.
            imu_bias_w (float):
                Standard deviation associated with uncertainty of the bias in
                inertial gyroscope sensor in [rad/s].
                Default: 1.745e-4 rad/s ≈ 0.01°/s.
            lid_p (float):
                Standard deviation associated with uncertainty of position
                estimate from LiDAR scan matching algorithm in [m].
                Default: 0.01 m.
            lid_r (float):
                Standard deviation associated with uncertainty of orientation
                estimate from LiDAR scan matching algorithm in [rad].
                Default: 8.727e-3 rad ≈ 0.5°.
        """

        # Process noise attributed to odometry
        std_imu_f = 0.05                # [m/s^2]
        std_imu_w = 10*np.pi/180        # [rad/s]
        std_imu_bias_f = 0.0005         # [m/s^2]
        std_imu_bias_w = 0.01*np.pi/180 # [rad/s]

        # Measurement noise attributed to LiDAR scans
        std_lid_p = 0.01          # [m] x and y position
        std_lid_r = 0.5*np.pi/180 # [rad] yaw angle

        # Constructs covariance arrays
        self.Q_imu = np.diag(np.array([std_imu_f]*3 + [std_imu_w]*3 + [std_imu_bias_f]*3 + [std_imu_bias_w]*3))**2
        self.R_lid = np.diag([std_lid_p, std_lid_p, std_lid_r])**2


    def initialize_arrays(self):
        """
        Initializes state and covariance arrays. Initial covariance standard
        deviations for the system state variables are pre-established here;
        however, feel free to modify these directly in the procedure.

        """

        self.p_est = np.zeros([self.n, 3])      # position estimates
        self.v_est = np.zeros([self.n, 3])      # velocity estimates
        self.q_est = np.zeros([self.n, 4])      # orientation estimates as quaternions
        self.del_u_est = np.zeros([self.n, 6])  # sensor bias estimates
        self.p_cov = np.zeros([self.n, 15, 15]) # covariance array at each timestep

        # Set initial values
        self.p_est[0] = np.zeros((1,3))
        self.v_est[0] = np.zeros((1,3))
        self.q_est[0] = self.imu_q[0,:]
        self.del_u_est[0] = np.zeros((1,6))

        # Initial uncertainties in standard deviations
        p_cov_p = 0.02*np.ones(3,)                # [m] position
        p_cov_v = 0.01*np.ones(3,)                # [m/s] velocity
        p_cov_q = np.array([1, 1, 20])*np.pi/180  # [rad] roll, pitch and yaw angles
        p_cov_bias_f = 0.02*np.ones(3,)           # [m/s^2] accelerometer biases
        p_cov_bias_w = 0.05*np.ones(3,)*np.pi/180 # [rad/s] gyroscope biases

        self.p_cov[0] = np.diag(np.hstack([p_cov_p, p_cov_v, p_cov_q, p_cov_bias_f, p_cov_bias_w]))**2

    def initialize_imu(self, stby_imu, stby_quaternions, stby=700, offset_quat=True, gravity_bias_est=False):
        """
        Loads, cleans and assigns IMU data to object attributes.

        Args:
            stby (int):
                Number of initial IMU readings (standby readings)
                before first movement. Default: 700.
            offset_quat (bool):
                If True, the quaternion readings will be offset such that
                initial heading is zero. Default: True.
            gravity_bias_est (bool):
                If True, the initial bias estimation, including estimating
                gravity component in the z-direction, will be done by averaging
                across the first ´stby´ number of IMU readings. Default: False
        """
        # Creating DataFrame table from lists   
        imu_df = pd.DataFrame(stby_imu)
        quaternion_df = pd.DataFrame(stby_quaternions)
        
        complex_imu_df = pd.concat([quaternion_df, imu_df], axis = 1)
        

        # Assign arrays for timestamp, linear acceleration and rotational velocity
        self.imu_t = np.round(complex_imu_df.timestamp.values / 1e9, 3) # converting to seconds with three decimal places
        self.imu_f = complex_imu_df[["a_x", "a_y", "a_z"]].values
        self.imu_w = complex_imu_df[["om_x", "om_y", "om_z"]].values
        self.imu_q = complex_imu_df[["q_w", "q_x", "q_y", "q_z"]].values
        n = self.imu_f.shape[0] # number of IMU readings

        if offset_quat:
            # Transform quaternions to Euler angles
            phi = np.ndarray((n, 3))
            for i in range(n):
                phi[i,:] = Quaternion(*self.imu_q[i,:]).normalize().to_euler()

            # Shift yaw angle such that the inital yaw equals zero
            inityaw = np.mean(phi[:stby,2])       # Estimated initial yaw
            
            """ Transformed yaw !!!"""
            self.tf_yaw = wraptopi(phi[:,2] - inityaw) 

            # Transform Euler angles back to quaternions
            phi[:,2] = self.tf_yaw
            for i in range(self.n):
                self.imu_q[i,:] = Quaternion(euler=phi[i,:]).normalize().to_numpy()

        if gravity_bias_est:
            imu_f_trans = np.ndarray((stby, 3))
            for i in range(stby):
                C_ns = Quaternion(*self.imu_q[i,:]).normalize().to_mat()
                imu_f_trans[i] = C_ns.dot(self.imu_f[i,:])

                self.g = np.mean(imu_f_trans,0) # bias + gravity
        else:
            self.g = np.array([0, 0, 9.80665]) # gravity
            
    # def single_imu(self, stby=700, offset_quat=True, gravity_bias_est=False):
    #     """
    #     Loads, cleans and assigns single IMU data to object attributes.
    #     """
        

    #     # Assign arrays for timestamp, linear acceleration and rotational velocity
    #     self.imu_t = np.round(complex_imu_df.timestamp.values / 1e9, 3) # converting to seconds with three decimal places
    #     self.imu_f = complex_imu_df[["a_x", "a_y", "a_z"]].values
    #     self.imu_w = complex_imu_df[["om_x", "om_y", "om_z"]].values
    #     self.imu_q = complex_imu_df[["q_w", "q_x", "q_y", "q_z"]].values
    #     n = self.imu_f.shape[0] # number of IMU readings

    #     if offset_quat:
    #         # Transform quaternions to Euler angles
    #         phi = np.ndarray((n, 3))
    #         for i in range(n):
    #             phi[i,:] = Quaternion(*self.imu_q[i,:]).normalize().to_euler()

    #         # Transform Euler angles back to quaternions
    #         phi[:,2] = self.tf_yaw
    #         for i in range(self.n):
    #             self.imu_q[i,:] = Quaternion(euler=phi[i,:]).normalize().to_numpy()

    #     if gravity_bias_est:
    #         imu_f_trans = np.ndarray((stby, 3))
    #         for i in range(stby):
    #             C_ns = Quaternion(*self.imu_q[i,:]).normalize().to_mat()
    #             imu_f_trans[i] = C_ns.dot(self.imu_f[i,:])

    #             self.g = np.mean(imu_f_trans,0) # bias + gravity
    #     else:
    #         self.g = np.array([0, 0, 9.80665]) # gravity
            

        
    
    def initialize_lidar(self):
        """
        Initializes global navigation frame with the point cloud from the first
        LiDAR scan, and also initializes the index of the next LiDAR scan and
        the position and heading state estimate associated with each LiDAR scan
        (important for sensor fusion -- see ´measurement_update´.)

        Returns:
            lid_i (int):
                Index of the next LiDAR scan to be processed.
            lid_state [3x0 (1D) Numpy array]:
                Initial position and heading state estimate associated with
                LiDAR scans.
        """

        # Initialize global navigation frame with first LiDAR scan
        if self.algorithm == "icp":
            self.gf = GlobalFrame(self.lid_r)
        if self.algorithm == "feature":
            self.gf = ScanFeature(self.lid_r[0,:], frame="global")

        #lid_i = 1                                    # index of next LiDAR scan
        lid_state = np.hstack([self.p_est[0,:2], 0]) # current position and heading state estimate

        return  lid_state


 
    def ekf_real_time(self, manager_data):
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
            imu_data = manager_data.get_current_imu()  # Fetch IMU data (accel, gyro, magnetometer)
            quaternion = manager_data.get_current_quaternions()  # Fetch quaternion data (orientation)
            lidar_data = manager_data.get_current_cloud()  # Fetch LIDAR data (point cloud with x, y, z)

            imu_data['acc'] = np.array(imu_data['acc'])
            imu_data['gyro'] = np.array(imu_data['gyro'])
            self.n += 1
            
            current_time = time.time()
            delta_t = current_time - prev_time
            prev_time = current_time

            # IMU calibration and state propagation (3D)
            f_km = imu_data['acc'] - del_u_km[:3].flatten()  # 3D acceleration
            w_km = imu_data['gyro'] - del_u_km[3:].flatten()  # 3D angular velocity
            
             # Update quaternion based on angular velocity
            quat = Quaternion(*q_km.flatten()).normalize()
            omega = np.array([0, *w_km])  # Extend angular velocity for quaternion multiplication
            q_dot = 0.5 * quat * Quaternion(*omega)
            q_check = (quat + q_dot * delta_t).normalize()
            
            # Compute rotation matrix from quaternion
            C_ns = q_check.to_mat()

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

            # LIDAR measurement update - Only x, y (2D)
            if lidar_data is not None and len(lidar_data) > 0:
                y_k = self.process_lidar_data(lidar_data)  # Extract position (x, y, z) from point cloud
                
                # Measurement matrix (maps state to measurement space)
                H = np.zeros((3, state_dim))
                H[:3, :3] = np.eye(3)  # Only position is observed
                
                new_state = SLAM.measurement_update(
                    y_k, p_check, v_check, q_check, del_u_check, p_cov_check, self.R_lid
                )
                p_check, v_check, q_check, p_cov_check, del_u_check, _ = new_state
                
                # Compute Kalman gain
                S = H.dot(p_cov_check).dot(H.T) + self.R_lid
                K = p_cov_check.dot(H.T).dot(np.linalg.inv(S))
                
                 # Update state and covariance
                residual = y_k.reshape(3, 1) - H.dot(np.vstack((p_check, v_check, q_check.flatten().reshape(-1, 1), del_u_check)))
                state_update = K.dot(residual)
                
                # Apply update
                p_check += state_update[:3]
                v_check += state_update[3:6]
                q_check = Quaternion(*q_check.flatten() + state_update[6:10].flatten()).normalize()
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
        Processes the LiDAR point cloud data and extracts the (x, y) position.
        Since the LiDAR only provides (x, y), we'll use the average position
        of the point cloud or another appropriate method.
        """
        
        # Konwersja listy na NumPy array
        lidar_data = np.array(lidar_data)

        # Example: Assuming lidar_data is a list of points (x, y)
        # We can use the average position of all points as the measurement
        x_vals = lidar_data[:, 0]  # x coordinates of LiDAR points
        y_vals = lidar_data[:, 1]  # y coordinates of LiDAR points

        # Compute the average position
        x_avg = np.mean(x_vals)
        y_avg = np.mean(y_vals)

        # Return the position (x, y) as a measurement (still 2D, but it's part of 3D state)
        return np.array([x_avg, y_avg]).reshape(2, 1)
        


    def postprocess(self):
        """
        Post-process the SLAM results.

        """

        if self.algorithm == "icp":
            self.pc_t = np.array([-self.gf.y_pc + self.gt_traj[0][0], self.gf.x_pc + self.gt_traj[0][1]]).T

        if self.algorithm == "feature":
            # Remove unsufficiently matched line segments
            rem_lines = []
            for i, line in enumerate(self.gf.lines):
                if line.it < 100:
                    rem_lines.append(line)

            for line in rem_lines:
                self.gf.lines.remove(line)

            # Transform LiDAR scans in global reference frame
            for line in self.gf.lines:
                line.x_start, line.y_start = -line.y_start + self.gt_traj[0][0], line.x_start + self.gt_traj[0][1]
                line.x_end, line.y_end = -line.y_end + self.gt_traj[0][0], line.x_end + self.gt_traj[0][1]

        # Transform final state estimates
        self.p_est[:,:2] = (np.array([[0,-1],[1,0]]).dot(self.p_est[:,:2].T) + self.gt_traj[0].reshape(2,1)).T
        self.v_est[:,:2] = np.array([[0,-1],[1,0]]).dot(self.v_est[:,:2].T).T


    def plot_results(self):
        """
        Plots the final SLAM results against the ground truth.

        """

        fig, ax = plt.subplots(figsize=(10,8))

        ax.plot(self.gt_traj[:,0], self.gt_traj[:,1], 'r-', markersize=0.5, label="Ground truth trajectory")
        ax.plot(self.gt_wall[:2,0], self.gt_wall[:2,1], 'k-', linewidth=0.5, label="Ground truth maze walls")
        ax.plot(self.gt_wall[:,0], self.gt_wall[:,1], 'k.', markersize=0.5, alpha=0.25)

        if self.algorithm == "icp":
            plot_idx = np.random.randint(self.pc_t.shape[0], size=round(self.pc_t.shape[0]/20))
            x_plot = self.pc_t[plot_idx,0]
            y_plot = self.pc_t[plot_idx,1]
            ax.plot(x_plot, y_plot, 'm.', markersize=1, label="Point cloud of walls")

        if self.algorithm == "feature":
            for i, line in enumerate(self.gf.lines):
                if i == 0:
                    ax.plot([line.x_start, line.x_end], [line.y_start, line.y_end], 'm--', linewidth=3, label="Estimated walls")
                else:
                    ax.plot([line.x_start, line.x_end], [line.y_start, line.y_end], 'm--', linewidth=3)

        ax.plot(self.p_est[:,0], self.p_est[:,1], 'b-.', lw=3, label="Estimated trajectory")
        ax.set_xlim(-0.2, 1.8)
        ax.set_ylim(-0.2, 1.4)
        plt.legend(loc="right", fontsize=9)
        plt.grid(alpha=0.5)
        start, end = ax.get_xlim()
        ax.xaxis.set_ticks(np.arange(start, end, 0.2))
        ax.set_title("SLAM results")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        plt.show() #it work in block mode


    @property
    def RMSE_traj(self):
        """
        Calculates and returns the root mean squared error (RMSE) for the
        deviation distance between the trajectory estimate and the ground truth.

        Returns:
            RMSE_traj (float):
                Root mean squared error (RMSE) for the trajectory error.
        """

        RMSE_traj = SLAM.RMSE(self.gt_traj, self.p_est[:,:2])
        return RMSE_traj


    @property
    def RMSE_wall_feature(self):
        """
        Calculates and returns the root mean squared error (RMSE) for the
        deviation distance between the mapping estimate and the ground truth,
        using the feature-based scan matching approach.

        Returns:
            RMSE_wall (float):
                Root mean squared error (RMSE) for the maze wall error.
        """

        x_wall = np.array([])
        y_wall = np.array([])

        for line in self.gf.lines:
            x_wall = np.hstack([x_wall, np.linspace(line.x_start, line.x_end, int(line.length*1000))])
            y_wall = np.hstack([y_wall, np.linspace(line.y_start, line.y_end, int(line.length*1000))])

        pc_t = np.vstack([x_wall, y_wall]).T

        RMSE_wall = SLAM.RMSE(self.gt_wall, pc_t)
        return RMSE_wall

    @property
    def RMSE_wall_icp(self):
        """
        Calculates and returns the root mean squared error (RMSE) for the
        deviation distance between the mapping estimate and the ground truth,
        using the ICP algorithm.

        Returns:
            RMSE_wall (float):
                Root mean squared error (RMSE) for the maze wall error.
        """

        RMSE_wall = SLAM.RMSE(self.gt_wall, self.pc_t)
        return RMSE_wall

    def plot_traj_error(self):
        """
        Plots deviation error of trajectory from start to finish.

        """

        RMSE_traj, traj_error = SLAM.RMSE(self.gt_traj, self.p_est[:,:2], return_error_arr=True)
        fig, ax = plt.subplots(figsize=(8,6))
        ax.plot(np.sqrt(traj_error), 'r', label="Residual distance from ground truth")
        ax.plot(RMSE_traj*np.ones((len(traj_error),)), 'k--', lw=1, label="RMSE = {0}m".format(round(RMSE_traj,4)))
        ax.set_title("Deviation error of trajectory")
        ax.set_xlabel("Time step k")
        ax.set_ylabel("Deviation error [m]")
        ax.set_xlim([0, len(traj_error)-1])
        ax.set_ylim([-0.01, 0.25])
        plt.legend(loc="upper left")
        plt.show()


    #####################
    ### Class methods ###
    #####################

    def feature_state(r_prev, r_new, prev_lid_state):
        """
        Obtains the position and orientation state of the system based on the
        feature-based (line) scan matching algorithm and LiDAR scans.

        Args:
            r_prev [1D Numpy array]:
                Range readings of the point cloud from the LiDAR scan in the
                previous time step.
            r_new [1D Numpy array]:
                Range readings of the point cloud from the LiDAR scan in the
                current time step.
            prev_lid_state [3x0 (1D) Numpy array]:
                1D Numpy array consisting of the x and y position and heading
                of the previous measurement update.

        Returns:
            y_k [1x3 Numpy array]:
                Position (x and y-coordinate) and orientation state of the
                scdystem.
        """

        # Unpack state from previous LiDAR scan
        x_prev = prev_lid_state[0]
        y_prev = prev_lid_state[1]
        head_prev = prev_lid_state[2]

        # Calculate estimated state from LiDAR scans using line feature algorithm
        x_lid, y_lid, head_lid = update_state(r_prev, r_new, x_prev, y_prev, head_prev)
        y_k = np.array([x_lid, y_lid, head_lid]).reshape(3,1)
        return y_k


    def icp_state(gf, r_new, prev_lid_state):
        """
        Obtains the position and orientation state of the system based on the
        ICP algorithm and LiDAR scans.

        Args:
            gf <GlobalFrame object>:
                Instance of GlobalFrame (specified in `icp`) representing the
                global navigation frame of the SLAM problem.
            r_new [1D Numpy array]:
                Range readings of the point cloud from the LiDAR scan in the
                current time step.
            prev_lid_state [3x0 (1D) Numpy array]:
                1D Numpy array consisting of the x and y position and heading
                of the previous measurement update.

        Returns:
            y_k [1x3 Numpy array]:
                Position (x and y-coordinate) and orientation state of the
                system.
        """

        # Unpack state from previous LiDAR scan
        prev_lid_pose = prev_lid_state[:2]
        head_prev = prev_lid_state[2]

        # Calculate estimated state from LiDAR scans using ICP algorithm
        x_lid, y_lid, head_lid = gf.next_scan(r_new, prev_lid_pose, head_prev)
        y_k = np.array([x_lid, y_lid, head_lid]).reshape(3,1)
        return y_k

    def measurement_update(y_k, p_check, v_check, q_check, del_u_check, p_cov_check, R_lid):
        """
        Performs the measurement update step of the Extended Kalman Filter for sensor fusion.
        
        Args:
            y_k [2x1 or 3x1 Numpy array]: Measurement from the sensor (LIDAR)
            p_check [6x1 Numpy array]: Predicted position and velocity
            v_check [6x1 Numpy array]: Predicted velocity
            q_check [4x1 Numpy array]: Quaternion representing orientation
            del_u_check [6x1 Numpy array]: Sensor biases
            p_cov_check [6x6 Numpy array]: Predicted covariance matrix
            R_lid [2x2 or 3x3 Numpy array]: Sensor noise covariance

        Returns:
            Updated state and covariance
        """
        
        # Jacobian H of the measurement model w.r.t. state
        H = np.zeros((6, 3))  # For 3D (x, y, z) measurements, the Jacobian should be (6, 3)
        
        # Compute H (Jacobian matrix) for 3D case (position, velocity)
        # For simplicity, we're assuming the relationship between position and measurements is linear
        H[:3, :] = np.eye(3)
        
        # Measurement residual (innovation)
        y_k_pred = p_check[:3]  # Predicted position
        y_residual = y_k - y_k_pred  # Residual between measurement and prediction

        # Compute Kalman gain
        S = H.dot(p_cov_check).dot(H.T) + R_lid
        K_k = p_cov_check.dot(H.T).dot(np.linalg.inv(S))
        
        # Update the state estimate
        p_check_new = p_check + K_k.dot(y_residual)
        p_cov_check_new = (np.eye(6) - K_k.dot(H)).dot(p_cov_check)
        
        return p_check_new, p_cov_check_new


    def state_space_model(imu_f, C_ns, delta_t):
        """
        Given state inputs and time step increment, this function returns the
        state transition matrix F and the noise gain matrix L required for the
        linearized state representation.

        Args:
            imu_f [3x1 Numpy array]:
                Original specific force vector from inertial accelerometer in
                [m/s^2] of the current time step.
            C_ns [3x3 Numpy array]:
                Direction cosine matrix that resolves the current orientation
                state to the navigation frame.
            delta_t (float):
                Time increment in [s] between the current time step and the
                previous one.

        Returns:
            F [6x6 Numpy array]:
                State transition matrix of the current time step.
            L [6x12 Numpy array]:
                Noise gain matrix of the current time step.
        """
        
        print("imu_f shape:", imu_f.shape)
        print("C_ns shape:", C_ns.shape)
        print("C_ns.dot(imu_f) shape:", (C_ns.dot(imu_f)).shape)
        
        if imu_f.ndim != 1 or imu_f.shape[0] != 3:
            raise ValueError("imu_f must be a 1D array with exactly 3 elements.")
        
        # F: State transition matrix (6x6) for 3D position and 3D velocity
        F = np.eye(6)

        # First 3 rows: Position update (linear part)
        F[:3, 3:6] = np.eye(3) * delta_t  # Position depends on velocity and time step

        # Next 3 rows: Velocity update (linear part)
        F[3:6, 3:6] = np.eye(3)  # Velocity is constant in the absence of forces

        # Calculate skew-symmetric matrix from force and orientation
        skew_matrix = skew_symmetric(C_ns.dot(imu_f).flatten()) * delta_t
        F[3:6, 0:3] = skew_matrix  # Velocity update depends on specific force

        # L: Noise gain matrix (6x12)
        L = np.zeros([6, 12])

        # Sensitivity of velocity to IMU accelerometer noise
        L[3:6, :3] = C_ns * delta_t

        # Sensitivity of position and velocity to IMU gyro and accelerometer noise
        L[3:6, 3:6] = C_ns * delta_t  # Sensitivity of velocity to gyro and accelerometer
        L[0:3, 6:9] = np.eye(3)  # Sensitivity of position to velocity errors
        L[3:6, 9:] = np.eye(3)  # Sensitivity of velocity to accelerometer errors

        return F, L




    def H():
        """
        Returns measurement model Jacobian matrix H.

        Returns:
            H [3x15 array]: measurement model Jacobian matrix.
        """

        H = np.zeros([3, 15])
        H[:3,:3] = np.eye(3)
        return H


    def RMSE(gt, est, return_error_arr=False):
        """
        Calculates and returns the root mean squared error (RMSE) for the
        deviation distance between the estimate and the ground truth.

        Args:
            gt [Mx2 Numpy array]:
                Numpy array with M rows of ground truth coordinates
                (x-coordinate: 1st column, y-coordinate: 2nd column).
            est [Nx2 Numpy array]:
                Numpy array with N rows of position estimate coordinates
                (x-coordinate: 1st column, y-coordinate: 2nd column).
            return_error_arr (bool):
                If True, the function also returns the list ´error´ containing
                the errors for each estimated point. Only really makes sense for
                trajectory.

        Returns:
            RMSE (float):
                Root mean squared error (RMSE) of the deviation error.
            error (list, opt.):
                List of deviation errors for each individual estimate point.
        """

        error = []
        for i in range(est.shape[0]):
            d = np.min(np.sum((gt - est[i,:])**2, axis=1))
            error.append(d)
        MSE = sum(error)/len(error)
        RMSE = np.sqrt(MSE)
        if return_error_arr:
            return RMSE, error
        return RMSE
