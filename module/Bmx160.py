import sys
import queue
sys.path.append('/home/karol/Desktop/repos/SLAM/lib')

import struct
from lib.DFRobot_BMX160 import BMX160
import smbus
import time

class Bmx160:

    def __init__(self, bus):
        self.sensor = BMX160(1)
        self.i2cbus = smbus.SMBus(bus)
        self.i2c_addr = 0x68
        self.data_queue = queue.Queue() 
        self.topic = "AHRS/bmx160"

        while not self.sensor.begin():
            time.sleep(2)

    def get_topic(self):
        return self.topic
    
    def save_to_queue(self):
            try:
                data = self.sensor.get_all_data()

                imu_data = [round(data[0], 2), round(data[1], 2), round(data[2], 2),               #magnetomert: xyz
                    round(data[3], 2), round(data[4], 2), round(data[5], 2),                       #gyroscope: xyz
                    round(data[6], 2), round(data[7], 2), round(data[8], 2)]                       #accelerometer: xyz
                
                #`bytearray`(format: 9 floatów)
                packed_data = bytearray(struct.pack('9f', *imu_data))

                self.data_queue.put(packed_data)

            except Exception as e:
                print(f"Error while saving BMX160 data to queue: {e}")

    def get_data_from_queue(self):
        if not self.data_queue.empty():
            return self.data_queue.get()
        else:
            #print("\nEmpty queue bmx160")
            return None
