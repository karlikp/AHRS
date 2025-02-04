import sys
import queue
import struct
import smbus
import time
import threading

sys.path.append('/home/karol/Desktop/repos/SLAM/lib')
from lib.DFRobot_BMX160 import BMX160
from .IMU_calibrator import IMU_calibrator

class Bmx160:

    
    def __init__(self, bus):
        self.mqtt_imu_queue = queue.Queue() 
        self.calibre_samples = [] 
        self.calibre_imu_ready = False
        self.calibre_ready_event = threading.Event()
        
        self.imu_calibrator = IMU_calibrator()
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
                'mag': [round(data[i], 2) for i in range(3)],               
                'gyro': [round(data[i], 2) for i in range(3, 6)],              
                 'acc': [round(data[i], 2) for i in range(6, 9)]                 
            }

            # Save the first 20 readings to calibre_samples
            if len(self.calibre_samples) < 20:
                self.calibre_samples.append(imu_data)
            else:
                self.calibre_imu_ready = True
                calibrate_imu = self.process_imu_data(imu_data)
                self.calibre_ready_event.set()
                # Serialize data into bytearray for MQTT
                packed_data = bytearray(struct.pack('9f', *[val for sublist in calibrate_imu.values() for val in sublist]))
                self.mqtt_imu_queue.put(packed_data)

        except Exception as e:
            print(f"Error while saving BMX160 data to queue: {e}")
            
    def process_imu_data(self, imu_data):
        
        self.imu_calibrator.calc_biases(self.calibre_samples)
        
        calibrate_imu = self.imu_calibrator.apply_calibration(imu_data)
        
        return calibrate_imu
        
    def get_data_from_queue(self):
        if not self.mqtt_imu_queue.empty():
            return self.mqtt_imu_queue.get()
        else:
            #debug to check queue capacity
            #print("\nEmpty queue bmx160")
            return None
    
    def get_calibre_imu_status(self):
        return self.calibre_imu_ready
    
    
