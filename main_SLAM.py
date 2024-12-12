import time
import threading
import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), 'module')))
from module import *

lidar = Lidar_LM1()

def lidar_reading():

    lidar.check_init()
    lidar.check_dirty()
    lidar_is_dirty = lidar.get_dirty()
    lidar.parsing_data()

if __name__ == "__main__":

    lidar_thread = threading.Thread(target=lidar_reading)  
    lidar_thread.start()
    lidar_thread.join()
   