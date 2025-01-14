import sys
import queue
sys.path.append('/home/karol/Desktop/repos/SLAM/lib')

import struct
from lib.DFRobot_BMX160 import BMX160
import smbus
import time
import threading

class Bmx160:

    
    
    def __init__(self, bus):
        self.current_imu = {}
        self.mqtt_imu_queue = queue.Queue() 
        self.calibre_data = [] 
        self.calibre_data_ready = False
        self.calibre_ready_event = threading.Event()
        
        self.sensor = BMX160(1)
        self.i2cbus = smbus.SMBus(bus)
        self.i2c_addr = 0x68
        self.topic = "AHRS/bmx160"

        while not self.sensor.begin():
            time.sleep(2)

    def get_topic(self):
        return self.topic
    
    def save_to_queue(self):
        
        try:
            data = self.sensor.get_all_data()

            # Prepare IMU data
            imu_data = {
                'mag': [round(data[i], 2) for i in range(3)],               # magnetometer: xyz
                'gyro': [round(data[i], 2) for i in range(3, 6)],               # gyroscope: xyz
                 'acc': [round(data[i], 2) for i in range(6, 9)]                 # accelerometer: xyz
            }
            
            
            
            #debug print
            #print(self.current_imu)

            # Save the first 100 readings to calibre_data
            if len(self.calibre_data) < 100:
                self.calibre_data.append(imu_data)
            else:
                self.current_imu = imu_data
                self.calibre_data_ready = True
                self.calibre_ready_event.set()

            # Serialize data into bytearray for MQTT
            packed_data = bytearray(struct.pack('9f', *[val for sublist in imu_data.values() for val in sublist]))
            self.mqtt_imu_queue.put(packed_data)

        except Exception as e:
            print(f"Error while saving BMX160 data to queue: {e}")
            
    def get_data_from_queue(self):
        if not self.mqtt_imu_queue.empty():
            return self.mqtt_imu_queue.get()
        else:
            print("\nEmpty queue bmx160")
            return None
        
    def get_current_imu(self):
        return self.current_imu
    
    # 0-2 xyz gyroscope; 3-5 xyz accelerometer
    def get_calibre_data(self):
        return self.calibre_data
    
    def get_calibre_status(self):
        return self.calibre_data_ready
    
