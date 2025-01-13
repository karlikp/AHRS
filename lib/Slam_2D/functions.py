from .rotations import Quaternion, skew_symmetric    
import numpy as np

def compute_quaternion_jacobian(C_ns, imu_f):
    """
    Compute the Jacobian of the velocity update with respect to the quaternion.

    Args:
        C_ns [3x3 Numpy array]: Rotation matrix from navigation to sensor frame.
        imu_f [3x1 Numpy array]: Specific force vector.
    
    Returns:
        Jacobian [3x4 Numpy array]: The Jacobian matrix.
    """
    # Compute the skew-symmetric matrix of the specific force
    skew_f = skew_symmetric(C_ns.dot(imu_f).flatten())
    
    # Quaternion Jacobian (partial derivatives)
    Jacobian = np.zeros((3, 4))
    Jacobian[:, 1:] = -2 * skew_f  # Map the vector part (x, y, z) to velocity dynamics

    return Jacobian