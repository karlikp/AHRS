import time
import threading

#sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), 'module')))
from module import *
from utils import *

if __name__ == "__main__":
    
    manager = Data_manager()
    
    manager.mqtt_connect()

    sensor_thread = threading.Thread(target = manager.sensor_reading)
    lidar_thread = threading.Thread(target = manager.lidar_reading)

    sensor_thread.start()
    lidar_thread.start()

    time.sleep(5)  

    # if manager.lidar_is_dirty:
    #     print("lidar too dirty, exit")
    #     exit()

    AHRS_mqtt = threading.Thread(target = manager.send_AHRS_data)
    Lidar_mqtt = threading.Thread(target = manager.send_Lidar_data)
    
    AHRS_mqtt.start()
    Lidar_mqtt.start()

    sensor_thread.join()
    lidar_thread.join()
    AHRS_mqtt.join()
    Lidar_mqtt.join()
    
   
    