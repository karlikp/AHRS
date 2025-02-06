import time
import board
import busio
import threading
import sys
import os
import numpy as np
from smbus2 import SMBus  


sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), 'module')))
from module import *
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), 'utils')))
from utils import *

class Data_manager:
    
    def __init__(self):
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.bus = SMBus(1)

        self.bmp388 = Bmp388(self.i2c)
        self.bmx160 = Bmx160(1)
        self.vl6180x = Vl6180x(self.i2c)
        self.vl53l1x = Vl53l1x(self.i2c)

        self.mqtt_client = Mqtt("test.mosquitto.org")

        self.AHRS_list = [self.bmp388, self.bmx160, self.vl6180x, self.vl53l1x] 
        self.lidar = Lidar_LM1()

        self.last_mean_calculation_time = time.time()

    def sensor_reading(self):
        print("Starting sensor reading thread")
        
        try:
            while True:
                current_time = time.time()

                self.bmp388.save_to_queue()
                self.bmx160.save_to_queue()

                self.vl6180x.collect_data()
                self.vl53l1x.collect_data()

            
                if current_time - self.last_mean_calculation_time >= 0.1:
                
                    self.vl6180x.save_to_queue()
                    self.vl53l1x.save_to_queue()

                    self.last_mean_calculation_time = current_time
        except KeyboardInterrupt:
            print("Program interrupted")



    def lidar_reading(self):   
        try:
            self.lidar.check_init()
            self.lidar.check_dirty()
            self.lidar.parsing_data()
        except Exception as e:
            print(f"Error in lidar_reading: {e}") 
        
    
    def send_AHRS_data(self):
    
        print(f"Sending AHRS")
        
        threads = []
        for sensor in self.AHRS_list:
            t = threading.Thread(target= self.AHRS_deamon, args=(sensor, sensor.get_topic()), daemon=True)
            threads.append(t)
            t.start()

        try:
            while any(t.is_alive() for t in threads):
                time.sleep(1)
        except KeyboardInterrupt:
            print("Stopping data transmission...")


    def send_Lidar_data(self):
        print(f"Sending Lidar")
        
        queue_topic_mapping = {
            self.lidar.mqtt_imu_queue: "Lidar/imu",
            self.lidar.mqtt_cloud_queue: "Lidar/cloud",
            self.lidar.mqtt_dirty_queue: "Lidar/dirty"
        }
        threads = []
        
        for queue, topic in queue_topic_mapping.items():
            queue_thread = threading.Thread(target=self.Lidar_deamon, args=(queue, topic), daemon=True)
            threads.append(queue_thread)
            queue_thread.start()
        
        variable_thread = threading.Thread(target=self.azimuth_deamon, args=(self.lidar,), daemon=True)
        threads.append(variable_thread)
        variable_thread.start()
        
        icp_matrix_thread = threading.Thread(target=self.send_icp_matrix_binary, args=(self.lidar,), daemon=True)
        threads.append(icp_matrix_thread)
        icp_matrix_thread.start()
        
        try:
            while any(t.is_alive() for t in threads):
                time.sleep(1)
        except KeyboardInterrupt:
            print("Stopping data transmission...")
            
    def AHRS_deamon(self, sensor, topic):
        
            while True:
                try:
                    data = sensor.get_data_from_queue()
                    
                    # check emptines
                    if data:
                        self.mqtt_client.client.publish(topic, data)
               
                except Exception as e:
                    print(f"Error occurred: {e}")

                time.sleep(0.1)  
                
    def azimuth_deamon(self, lidar):
       
        while True:
            try:
                azimuth_value = lidar.azimuth  
                self.mqtt_client.client.publish("Lidar/azimuth", azimuth_value)  

            except Exception as e:
                print(f"send azimuth data exception: {e}")
                return  

            time.sleep(0.5)  
            
    
    def send_icp_matrix_binary(self, lidar):
        
        while True:
            try:

                # Pobierz aktualny timestamp
                timestamp = lidar.matrix_timestamp  # 8 bajtów
                timestamp_bytes = np.array([timestamp], dtype=np.uint64).tobytes()

                # Konwersja macierzy do bytes
                matrix_bytes = lidar.tf_matrix.astype(np.float32).tobytes()

                # Połączenie timestamp + macierz
                full_payload = timestamp_bytes + matrix_bytes

                # Publikacja na MQTT
                self.mqtt_client.client.publish("Lidar/icp_tf_matrix", full_payload)
                
                time.sleep(0.5)

            except Exception as e:
                print(f"send icp_matrix_binary exception: {e}")
            
        
     
    def Lidar_deamon(self, data_pack, topic):
        
            while True:
                try:
                    while data_pack.empty():
                        time.sleep(0.5)
        
                    data = data_pack.get_nowait()  # Load data without block
                    self.mqtt_client.client.publish(topic, data)

                except Exception as e:
                    print(f"send lidar data exception: {e}")
                    return
                time.sleep(0.5) 
        
    def mqtt_connect(self):
            self.mqtt_client.client.connect("test.mosquitto.org", 1883, 60)
            
    def lidar_is_dirty(self):
        return self.lidar.get_dirty()
    
    def get_calibre_imu_status(self):
        return self.bmx160.get_calibre_imu_status()
    
    def get_transformation_matrix(self):
        return self.lidar.get_transformation_matrix()
    
    def get_lidar_calibrate(self):
        return self.lidar.lidar_calibrate()
    
            