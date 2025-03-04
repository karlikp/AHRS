import time
import threading
import os
import sys

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), 'module')))
from module import *
from utils import *

if __name__ == "__main__":
    
    data_manager = Data_manager()
    
    data_manager.mqtt_connect()

    sensor_thread = threading.Thread(target = data_manager.sensor_reading)
    sensor_thread.start()
    
    lidar_thread = threading.Thread(target=data_manager.lidar_reading)
    lidar_thread.start()
    
    data_manager.bmx160.calibre_ready_event.wait() # Waiting for IMU calibration
    #data_manager.lidar.calibre_ready_event.wait() # Waiting for Lidar calibration
    time.sleep(0.5)
    
    AHRS_mqtt = threading.Thread(target = data_manager.send_AHRS_data)
    Lidar_mqtt = threading.Thread(target = data_manager.send_Lidar_data)
    
    AHRS_mqtt.start()
    Lidar_mqtt.start()

    sensor_thread.join()
    lidar_thread.join()
    AHRS_mqtt.join()
    Lidar_mqtt.join()
