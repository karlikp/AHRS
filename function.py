import sys
import time
sys.path.append("/home/karol/Desktop/repos/SLAM/module")
from module.Mqtt import Mqtt

def clear_files():
    files_to_clear = [
        "/home/karol/Desktop/repos/SLAM/data/package/bmp388.txt",
        "/home/karol/Desktop/repos/SLAM/data/package/bmx160.txt",
        "/home/karol/Desktop/repos/SLAM/data/package/distances.txt",
        "/home/karol/Desktop/repos/SLAM/data/package/lidar_cloud.txt",
        "/home/karol/Desktop/repos/SLAM/data/package/lidar_imu.txt",
        "/home/karol/Desktop/repos/SLAM/data/package/lidar_dirty.txt"

    ]
    
    # Clear all files in the list
    for file_path in files_to_clear:
        open(file_path, "w").close()

def send_package():
    # Lista plików i odpowiadających im tematów MQTT
    file_topic_pairs = [
        ("/home/karol/Desktop/repos/SLAM/data/package/bmp388.txt", "file_transfer/bmp388"),
        ("/home/karol/Desktop/repos/SLAM/data/package/bmx160.txt", "file_transfer/bmx160"),
        ("/home/karol/Desktop/repos/SLAM/data/package/distances.txt", "file_transfer/distances"),
        ("/home/karol/Desktop/repos/SLAM/data/package/lidar_cloud.txt", "file_transfer/lidar_cloud"),
        ("/home/karol/Desktop/repos/SLAM/data/package/lidar_imu.txt", "file_transfer/lidar_imu"),
        ("/home/karol/Desktop/repos/SLAM/data/package/lidar_dirty.txt", "file_transfer/lidar_dirty")
    ]

    # Tworzenie instancji Mqtt dla każdego pliku w liście
    topics = [Mqtt("test.mosquitto.org", file_path, topic) for file_path, topic in file_topic_pairs]

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
