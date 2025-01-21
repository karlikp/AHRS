import time
import threading

#sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), 'module')))
from module import *
from utils import *

if __name__ == "__main__":
    
    data_manager = Data_manager()
    
    data_manager.mqtt_connect()

    sensor_thread = threading.Thread(target = data_manager.sensor_reading)
    sensor_thread.start()
    
    data_manager.bmx160.calibre_ready_event.wait() # Waiting for IMU data to calibration
    
    lidar_thread = threading.Thread(target = data_manager.lidar_reading)
    lidar_thread.start()

    #time.sleep(3)
     
    #test_libpoint = threading.Thread(target = icp_process, args = (data_manager,))
    #test_libpoint.start()    
    # #SLAM_thread = threading.Thread(target = slam_process, args = (data_manager,))
    
    # #AHRS_mqtt = threading.Thread(target = data_manager.send_AHRS_data)
    # #Lidar_mqtt = threading.Thread(target = data_manager.send_Lidar_data)
    
    # #SLAM_thread.start()
    # #AHRS_mqtt.start()
    # #Lidar_mqtt.start()

    sensor_thread.join()
    lidar_thread.join()
    #test_libpoint.join()
    # #AHRS_mqtt.join()
    # #Lidar_mqtt.join()
    # #SLAM_thread.join()
    
   
    