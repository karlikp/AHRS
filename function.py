import sys
import time
sys.path.append("/home/karol/Desktop/repos/SLAM/module")
from module.Mqtt import Mqtt

def clear_files():
    open("/home/karol/Desktop/repos/SLAM/data/current/bmp388.txt", "w").close()
    open("/home/karol/Desktop/repos/SLAM/data/package/bmp388.txt", "w").close()

    open("/home/karol/Desktop/repos/SLAM/data/current/bmx160.txt", "w").close()
    open("/home/karol/Desktop/repos/SLAM/data/package/bmx160.txt", "w").close()

    open("/home/karol/Desktop/repos/SLAM/data/current/imu_data.txt", "w").close()
    open("/home/karol/Desktop/repos/SLAM/data/current/cloud_data.txt", "w").close()

    open("/home/karol/Desktop/repos/SLAM/data/package/dirty_percentage.txt", "w").close()
    open("/home/karol/Desktop/repos/SLAM/data/package/imu_data.txt", "w").close()
    open("/home/karol/Desktop/repos/SLAM/data/package/cloud_data.txt", "w").close()

    open("/home/karol/Desktop/repos/SLAM/data/current/vl53l1x.txt", "w").close()
    open("/home/karol/Desktop/repos/SLAM/data/package/vl53l1x.txt", "w").close()

    open("/home/karol/Desktop/repos/SLAM/data/current/vl6180x.txt", "w").close()
    open("/home/karol/Desktop/repos/SLAM/data/package/vl6180x.txt", "w").close()

def get_data_from_file():

    #     # Create an instance of the Mqtt class
    # sender = Mqtt(broker="mqtt.eclipse.org", topic="file_transfer/topic")

    # # Connect to the MQTT broker and send the file
    # sender.connect()

    # # Path to the file to be sent
    # file_path = "/home/karol/Desktop/repos/SLAM/data/package/bmp388.txt"  # Change to the appropriate file path
    # sender.send_file(file_path)

    topics = []

    topics.append(Mqtt("mqtt.eclipse.org", "/home/karol/Desktop/repos/SLAM/data/package/bmp388.txt", "file_transfer/bmp388")) 
    topics.append(Mqtt("mqtt.eclipse.org", "/home/karol/Desktop/repos/SLAM/data/package/bmx160.txt", "file_transfer/bmx160"))
    topics.append(Mqtt("mqtt.eclipse.org", "/home/karol/Desktop/repos/SLAM/data/package/distances.txt", "file_transfer/distances"))
    topics.append(Mqtt("mqtt.eclipse.org", "/home/karol/Desktop/repos/SLAM/data/package/lidar_cloud.txt", "file_transfer/lidar_cloud"))
    topics.append(Mqtt("mqtt.eclipse.org", "/home/karol/Desktop/repos/SLAM/data/package/lidar_imu.txt", "file_transfer/lidar_imu"))
    topics.append(Mqtt("mqtt.eclipse.org", "/home/karol/Desktop/repos/SLAM/data/package/lidar_dirty.txt", "file_transfer/lidar_dirty"))

    while True:
        time.sleep(5)

        for topic in topics:

            topic.connect()

            file_path = topic.get_file_path()
            topic.send_file(file_path)
    

    