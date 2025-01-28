import numpy as np
import matplotlib.pyplot as plt
import time
# from numpy.linalg import inv
from .functions import *


def ekf_real_time(manager_data, imu_calibrator, semaphore, calibrated_matrix):
    """
    Extended Kalman Filter adapted for real-time sensor fusion
    with LiDAR (providing x, y) and IMU (accelerometer, gyroscope, magnetometer).
    """

    # Initialize state and covariance (3D position, 3D velocity, and biases)
    n = 10000  # Estimate maximum number of iterations or extend dynamically
    state_dim = 3 + 3 + 4 + 6  # 3 position + 3 velocity + 4 quaternion + 6 biases

    # Arrays to store states and uncertainties
    p_est = np.zeros((n, 3))      # Position estimates
    v_est = np.zeros((n, 3))      # Velocity estimates
    q_est = np.zeros((n, 4))      # Orientation estimates (quaternions)
    del_u_est = np.zeros((n, 6))  # Sensor bias estimates
    p_cov = np.zeros((n, state_dim, state_dim))  # Covariance arrays

    # Initialize first state
    p_est[0] = np.zeros(3)
    v_est[0] = np.zeros(3)
    q_est[0] = [1, 0, 0, 0]  # Quaternion (identity rotation)
    del_u_est[0] = np.zeros(6)
    p_cov[0] = np.eye(state_dim) * 0.1

    # Initialize noise covariance matrices
    Q_imu = np.eye(6) * 0.01  # IMU process noise (accel and gyro)
    R_lid = np.eye(3) * 0.05  # LiDAR measurement noise (x, y, z)

    print("Initiating real-time Kalman filter loop...")
    prev_time = time.time()
    k = 0  # Step counter

    while k < n - 1:
        # Get IMU and LiDAR data in real-time
        raw_imu_data = manager_data.get_current_imu()
        quaternions = manager_data.get_current_quaternions()

        # Apply calibration to raw IMU data
        imu_data = imu_calibrator.apply_calibration(raw_imu_data)
        imu_data['acc'] = np.array(imu_data['acc'])
        imu_data['gyro'] = np.array(imu_data['gyro'])
        g = np.array([0, 0, -9.81]).reshape(3, 1)  # Gravitational acceleration

        current_time = time.time()
        delta_t = current_time - prev_time
        prev_time = current_time

        # Assign previous state
        p_km = p_est[k].reshape(3, 1)
        v_km = v_est[k].reshape(3, 1)
        q_km = q_est[k].reshape(4, 1)
        del_u_km = del_u_est[k].reshape(6, 1)
        p_cov_km = p_cov[k]

        # IMU calibration and state propagation
        f_km = np.expand_dims(imu_data['acc'] - del_u_km[:3].flatten(), axis=1)  # Acceleration
        w_km = imu_data['gyro'] - del_u_km[3:].flatten()  # Angular velocity

        q_check = Quaternion(*quaternions).normalize()
        C_ns = q_check.to_mat()

        # Wait for LiDAR data
        semaphore.acquire()
        transformation_matrix = manager_data.get_transformation_matrix()
        
        corrected_T = np.linalg.inv(calibrated_matrix) @ transformation_matrix
        y_k = compute_y_k(corrected_T)

        # Propagate state
        p_check = p_km + delta_t * v_km + (delta_t ** 2 / 2) * (C_ns.dot(f_km) - g)
        v_check = v_km + delta_t * (C_ns.dot(f_km) - g)
        del_u_check = del_u_km

        # Linearize motion model
        F, L = state_space_model(f_km, C_ns, delta_t)

        # Propagate uncertainty
        p_cov_check = F.dot(p_cov_km).dot(F.T) + L.dot(Q_imu).dot(L.T)

        # Perform measurement update
        H = np.zeros((3, state_dim))
        H[:3, :3] = np.eye(3)  # Position is directly observed

        new_state = measurement_update(
            y_k, p_check, v_check, q_check, del_u_check, p_cov_check, R_lid
        )
        p_check, v_check, q_check, del_u_check, p_cov_check = new_state

        # Store updated state
        p_est[k + 1] = p_check.flatten()
        v_est[k + 1] = v_check.flatten()
        q_est[k + 1] = q_check.to_numpy()
        del_u_est[k + 1] = del_u_check.flatten()
        p_cov[k + 1] = p_cov_check

        # Print real-time position
        print(f"Real-time Position: x = {p_check[0, 0]:.2f}, y = {p_check[1, 0]:.2f}, z = {p_check[2, 0]:.2f}")

        k += 1
        time.sleep(0.5)

    print("EKF real-time loop finished.")

    # Return the final states and uncertainties
    return p_est[:k + 1], v_est[:k + 1], q_est[:k + 1], del_u_est[:k + 1], p_cov[:k + 1]



def measurement_update(y_k, p_check, v_check, q_check, del_u_check, p_cov_check, R_lid):
    
    state_dim = 16
    measurement_dim = 3  
    
    # Measurement model Jacobian (H maps state to measurement)
    H = np.zeros((measurement_dim, state_dim))
    H[:3, :3] = np.eye(3)  

    # Measurement residual 
    y_k_pred = p_check  
    y_residual = y_k - y_k_pred 
        
    # Compute Kalman gain
    S = H.dot(p_cov_check).dot(H.T) + R_lid
    K_k = p_cov_check.dot(H.T).dot(np.linalg.inv(S))

    # Update state estimate
    state_update = K_k.dot(y_residual)

    # Extract updated states
    p_check_new = p_check + state_update[:3]
    v_check_new = v_check + state_update[3:6]
    q_check_new = Quaternion(*( q_check.to_numpy() + state_update[6:10].flatten())).normalize()
    del_u_check_new = del_u_check + state_update[10:]

    # Update covariance
    p_cov_check_new = (np.eye(state_dim) - K_k.dot(H)).dot(p_cov_check)

    return p_check_new, v_check_new, q_check_new, del_u_check_new, p_cov_check_new


def state_space_model(imu_f, C_ns, delta_t):
    
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

def compute_y_k(transformation_matrix):
    """
    Compute the current position y_k in global coordinates using the transformation matrix.

    :param transformation_matrix: 4x4 transformation matrix (from LiDAR data)
    :return: y_k (3x1 position vector in global coordinates)
    """
    print(f"Transformation matrix: {transformation_matrix}")

    if transformation_matrix is None:
        raise ValueError("Transformation matrix is None. Ensure it is initialized properly.")

    if transformation_matrix.shape != (4, 4):
        raise ValueError(f"Invalid transformation matrix shape: {transformation_matrix.shape}. Expected (4, 4).")

    # Assume the position in the local frame is at the origin
    local_position = np.array([[0], [0], [0], [1]])  # Homogeneous coordinates

    # Transform the local position to the global frame
    global_position = transformation_matrix @ local_position

    # Return the 3D position (ignoring the homogeneous coordinate)
    return global_position[:3]

