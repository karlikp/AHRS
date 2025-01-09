import time
import threading

#sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), 'module')))
from module import *
from utils import *

if __name__ == "__main__":
    
    data_manager = Data_manager()
    
    data_manager.mqtt_connect()

    sensor_thread = threading.Thread(target = data_manager.sensor_reading)
    lidar_thread = threading.Thread(target = data_manager.lidar_reading)

    sensor_thread.start()
    lidar_thread.start()

    time.sleep(5)  

    # if manager.lidar_is_dirty:
    #     print("lidar too dirty, exit")
    #     exit()

    SLAM_thread = threading.Thread(target = slam_process, args = (data_manager,))
    #AHRS_mqtt = threading.Thread(target = data_manager.send_AHRS_data)
    #Lidar_mqtt = threading.Thread(target = data_manager.send_Lidar_data)
    
    SLAM_thread.start()
    #AHRS_mqtt.start()
    #Lidar_mqtt.start()

    sensor_thread.join()
    lidar_thread.join()
    #AHRS_mqtt.join()
    #Lidar_mqtt.join()
    SLAM_thread.join()
    
   
    