import numpy as np
import matplotlib.pyplot as plt
import time
# from numpy.linalg import inv
from .functions import *


def ekf_real_time(manager_data, imu_calibrator, semaphore):
    """
    Extended Kalman Filter adapted for real-time sensor fusion
    with LIDAR (providing only x, y), quaternions, and IMU (accelerometer, gyroscope, magnetometer).
    """
    
    
    # Initialize state and covariance (3D position, 3D velocity, and biases)
    p_km = np.zeros((3, 1))  # Initial position (x, y, z)
    v_km = np.zeros((3, 1))  # Initial velocity (vx, vy, vz)
    q_km = np.array([1, 0, 0, 0]).reshape(4, 1)  # Initial quaternion (no rotation)
    del_u_km = np.zeros((6, 1))  # Sensor biases (accelerometer, gyroscope)

    n = 0

    # State vector now includes position, velocity, quaternion, and biases
    state_dim = 3 + 3 + 4 + 6  # 3 position + 3 velocity + 4 quaternion + 6 biases
    p_cov_km = np.eye(state_dim) * 0.1  # Initialize covariance matrix
    
    # Initialize the variable `p_cov_check`
    p_cov_check = p_cov_km.copy()  # Make a copy of the initial covariance matrix

    # Initialize noise covariance matrices
    Q_imu = np.eye(6) * 0.01  # IMU process noise (accel and gyro)
    R_lid = np.eye(3) * 0.05  # LiDAR measurement noise (x, y, z)

        
    # Start real-time loop
    print("Initiating real-time Kalman filter loop...")
    prev_time = time.time()

    while True:
        # Get IMU and LiDAR data in real-time
        raw_imu_data = manager_data.get_current_imu()  # Fetch IMU data (accel, gyro, magnetometer)
        quaternions = manager_data.get_current_quaternions()  # Fetch quaternion data (orientation)

        # Apply calibration to raw IMU data
        imu_data = imu_calibrator.apply_calibration(raw_imu_data) 
                
        imu_data['acc'] = np.array(imu_data['acc'])
        imu_data['gyro'] = np.array(imu_data['gyro'])
        g = np.array([0, 0, -9.81]).reshape(3, 1)  # Przyspieszenie ziemskie [m/s²]
        n += 1
        
        current_time = time.time()
        delta_t = current_time - prev_time
        prev_time = current_time
        
        # IMU calibration and state propagation (3D)
        f_km = np.expand_dims(imu_data['acc'] - del_u_km[:3].flatten(), axis=1)  # 3D acceleration jako (3, 1)
        w_km = imu_data['gyro'] - del_u_km[3:].flatten()  # 3D angular velocity
        
        # Use quaternion from sensor data instead of computing from IMU
        q_check = Quaternion(*quaternions).normalize()
        
        # Compute rotation matrix from quaternion
        C_ns = q_check.to_mat()
        
        # Get transfomation matrix after compute it
        semaphore.acquire()
        transformation_matrix = manager_data.get_transformation_matrix()
        
        # Compute y_k using transformation matrix
        y_k = compute_y_k(transformation_matrix)
        
        # Apply transformation to position and velocity
        p_homogeneous = np.vstack((p_km, [[1]]))  # Convert position to homogeneous coordinates
        v_homogeneous = np.vstack((v_km, [[1]]))  # Convert velocity to homogeneous coordinates

        p_transformed = transformation_matrix @ p_homogeneous
        v_transformed = transformation_matrix @ v_homogeneous
        
        # Usuń wymiar homogeniczny (ostatni wiersz) przed użyciem w równaniach
        p_transformed = p_transformed[:3]  # Pobierz tylko pierwsze trzy elementy
        v_transformed = v_transformed[:3]  # Pobierz tylko pierwsze trzy elementy

        # Propagate state based on IMU data and transformation matrix from lidar
        p_check = p_transformed + delta_t * v_transformed + (delta_t ** 2 / 2) * (C_ns.dot(f_km) - g)
        v_check = v_transformed + delta_t * (C_ns.dot(f_km) - g)
        del_u_check = del_u_km

        # Linearize motion model
        F, L = state_space_model(f_km, C_ns, delta_t)

        # Ensure F has the correct size
        if F.shape != (state_dim, state_dim):
            raise ValueError(f"Shape mismatch: F should be {state_dim}x{state_dim}, but got {F.shape}")
        
    
        if True:
            # Measurement matrix (maps state to measurement space)
            H = np.zeros((3, state_dim))
            H[:3, :3] = np.eye(3)  # Only position is observed
            
            new_state = measurement_update(
                y_k, p_check, v_check, q_check, del_u_check, p_cov_check, R_lid
            )
            p_check, v_check, q_check, del_u_check, p_cov_check = new_state
            
            # Compute Kalman gain
            S = H.dot(p_cov_check).dot(H.T) + R_lid
            
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
            
            # Propagate uncertainty (3D state)
            p_cov_check = F.dot(p_cov_km).dot(F.T) + L.dot(Q_imu).dot(L.T)
            
            # Update covariance
            p_cov_check = (np.eye(state_dim) - K.dot(H)).dot(p_cov_check)
            
        

        # Store updated state
        p_km, v_km, q_km, del_u_km, p_cov_km = p_check, v_check, q_check, del_u_check, p_cov_check

        # Print real-time position (3D)
        print(f"{imu_data}")
        print(f"Real-time Position: x = {p_km[0, 0]:.2f}, y = {p_km[1, 0]:.2f}, z = {p_km[2, 0]:.2f}")
        
        time.sleep(0.5)


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

