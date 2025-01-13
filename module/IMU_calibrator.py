import numpy as np
import time

class IMU_calibrator:
    def __init__(self):
        self.acc_bias = None
        self.gyro_bias = None
        self.mag_bias = None
        self.g = 9.81  # Gravitational acceleration [m/s^2]

    def calibrate_imu(self, imu_data_samples):
        """
        Calibrates the IMU based on a series of stationary measurements.

        Args:
            imu_data_samples (list of dict): List of dictionaries containing IMU readings, e.g.,
                                             [{'acc': [...], 'gyro': [...], 'mag': [...]}]

        Returns:
            dict: Biases and other calibration parameters for accelerometer, gyroscope, and magnetometer.
        """
        # Extract data samples for each sensor
        acc_samples = np.array([data['acc'] for data in imu_data_samples])
        gyro_samples = np.array([data['gyro'] for data in imu_data_samples])
        mag_samples = np.array([data['mag'] for data in imu_data_samples])

        # Accelerometer calibration: Compute mean readings and determine bias
        acc_mean = np.mean(acc_samples, axis=0)  # Average accelerometer readings
        acc_bias = acc_mean - np.array([0, 0, self.g])  # Assuming gravity acts only in the Z-axis

        # Gyroscope calibration: Compute mean readings and set bias
        gyro_mean = np.mean(gyro_samples, axis=0)  # Average gyroscope readings
        gyro_bias = gyro_mean  # In stationary state, gyroscope should ideally return zero

        # Magnetometer calibration: Compute mean readings and set bias
        mag_mean = np.mean(mag_samples, axis=0)  # Average magnetometer readings
        mag_bias = mag_mean  # Magnetometer offset correction (hard iron bias)

        # Store the biases
        self.acc_bias = acc_bias
        self.gyro_bias = gyro_bias
        self.mag_bias = mag_bias

        # Return calibration results
        return {
            "acc_bias": acc_bias,
            "gyro_bias": gyro_bias,
            "mag_bias": mag_bias,
        }

    def apply_calibration(self, imu_data):
        """
        Applies calibration to IMU readings.

        Args:
            imu_data (dict): IMU readings, e.g., {'acc': [...], 'gyro': [...], 'mag': [...]}

        Returns:
            dict: Calibrated IMU data.
        """
        calibrated_data = {}

        # Correct accelerometer readings
        if self.acc_bias is not None:
            calibrated_data['acc'] = np.array(imu_data['acc']) - self.acc_bias
        else:
            calibrated_data['acc'] = np.array(imu_data['acc'])

        # Correct gyroscope readings
        if self.gyro_bias is not None:
            calibrated_data['gyro'] = np.array(imu_data['gyro']) - self.gyro_bias
        else:
            calibrated_data['gyro'] = np.array(imu_data['gyro'])

        # Correct magnetometer readings
        if self.mag_bias is not None:
            calibrated_data['mag'] = np.array(imu_data['mag']) - self.mag_bias
        else:
            calibrated_data['mag'] = np.array(imu_data['mag'])

        return calibrated_data
    
    def calibrate_imu_once(self, manager_data):
        """
        Performs IMU calibration when the vehicle is stationary.
        """
        print("Starting IMU calibration...")

        # Collect stationary IMU samples
        while (not manager_data.get_calibre_imu_status()):
            time.sleep(1)
            
        stationary_samples = manager_data.get_calibre_imu()

        # Perform calibration using the collected samples
        calibration_result = self.calibrate_imu(stationary_samples)
        print("IMU calibration completed:", calibration_result)
      
