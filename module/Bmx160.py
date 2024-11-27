import sys
import os
#sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'libs')))
sys.path.append('/home/karol/Desktop/repos/SLAM/lib')
#sys.path.append('../')


from lib.DFRobot_BMX160 import BMX160
import smbus
import time

class Bmx160:

    def __init__(self, bus):
        self.sensor = BMX160(1)
        self.i2cbus = smbus.SMBus(bus)
        self.i2c_addr = 0x68
        open("/home/karol/Desktop/repos/SLAM/data/current/bmx160.txt", "w").close()
        open("/home/karol/Desktop/repos/SLAM/data/package/bmx160.txt", "w").close()

    def save_to_file(self):
        data = self.sensor.get_all_data()
        time.sleep(1)
    def save_bmx160_data(self, data):

        file_path1 = "/home/karol/Desktop/repos/SLAM/data/current/bmx160.txt"
        file_path2 = "/home/karol/Desktop/repos/SLAM/data/package/bmx160_additional.txt"

        try:
            imu_data = f"magn: x: {data[0]:.2f} uT, y: {data[1]:.2f} uT, z: {data[2]:.2f} uT"
            imu_data += f"gyro  x: {data[3]:.2f} g, y: {data[4]:.2f} g, z: {data[5]:.2f} g"
            imu_data += f"accel x: {data[6]:.2f} m/s^2, y: {data[7]:.2f} m/s^2, z: {data[8]:.2f} m/s^2"

            with open(file_path1, "w") as imu_file1, open(file_path2, "a") as imu_file2:
                imu_file1.write(imu_data)
                imu_file2.write(imu_data)
        except Exception as e:
            print(f"Error while saving BMX160 data: {e}")
                    
        except RuntimeError:
            print("BMP388 Error reading data")