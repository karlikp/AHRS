import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'libs')))
sys.path.append('../')
from libs.DFRobot_BMX160 import BMX160
import smbus

class Bmx160:

    def __init__(self, bus):
            self.sensor = BMX160(1)
            self.i2cbus = smbus.SMBus(bus)
            self.i2c_addr = 0x68

    def save_to_file(self):
        data = self.sensor.get_all_data()
        with open("/home/karol/Desktop/repos/SLAM/data/imu.txt", "w") as imu_file:
            try:
                imu_data = f"magn: x: {data[0]:.2f} uT, y: {data[1]:.2f} uT, z: {data[2]:.2f} uT"
                imu_data += f"gyro  x: {data[3]:.2f} g, y: {data[4]:.2f} g, z: {data[5]:.2f} g"
                imu_data+= f"accel x: {data[6]:.2f} m/s^2, y: {data[7]:.2f} m/s^2, z: {data[8]:.2f} m/s^2"
                imu_file.write(imu_data)
                
            except RuntimeError:
                    print("BMP388 Error reading data")