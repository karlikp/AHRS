import sys
import time
sys.path.append("/home/karol/Desktop/repos/SLAM/module")
from module.Mqtt import Mqtt

def send_package():
    # Lista plików i odpowiadających im tematów MQTT
    file_topic_pairs = [
        ("/home/karol/Desktop/repos/SLAM/data/package/bmp388.txt", "AHRS/bmp388"),
        ("/home/karol/Desktop/repos/SLAM/data/package/bmx160.txt", "AHRS/bmx160"),
        ("/home/karol/Desktop/repos/SLAM/data/package/distances.txt", "AHRS/distances")
        #("/home/karol/Desktop/repos/SLAM/data/package/lidar_cloud.txt", "Lidar/lidar_cloud"),
        #("/home/karol/Desktop/repos/SLAM/data/package/lidar_imu.txt", "file_transfer/lidar_imu"),
        #("/home/karol/Desktop/repos/SLAM/data/package/lidar_dirty.txt", "file_transfer/lidar_dirty")
    ]
    bmp388 = Bmp388("AHRS/bmp388")
    bmx160 = Bmx160("AHRS/bmx160")

    #First append vl53 then append vl61
    vl53l1x = Vl53l1x("AHRS/distances")
    vl6180x = Vl6180x("AHRS/distances")
    
    #Lidar service different topics 
    lidar = Lidar_LM1("")

    # Tworzenie instancji Mqtt dla każdego pliku w liście
    topics = [Mqtt("192.168.50.210", file_path, topic) for file_path, topic in file_topic_pairs]

    try:
        # Połącz się ze wszystkimi brokerami na początku
        for topic in topics:
            topic.connect()  # Establish persistent connection

        # Główna pętla
        while True:
            for topic in topics:
                try:
                    topic.send_file()  # Send the file
                    topic.clear_file()  # Clear the file after successful send
                except Exception as e:
                    print(f"Error processing topic {topic.topic}: {e}")
            time.sleep(5)  # Wait before next iteration

    except KeyboardInterrupt:
        print("Shutting down...")
