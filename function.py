import sys
import time
sys.path.append("/home/karol/Desktop/repos/SLAM/module")
from module.Mqtt_module import Mqtt
import threading
import time



def send_AHRS_data(AHRS, mqtt_client):
    
    def send_data(sensor, topic):
        while True:
            try:
                data = sensor.get_data_from_queue()
                
                # check emptines
                if data:
                    mqtt_client.client.publish(topic, data)
                    #print(f"Sent data: {data}")
                # else:
                #     print("No data to send. Skipping this cycle.")

            except Exception as e:
                print(f"Error occurred: {e}")

            time.sleep(0.1)  

    # Create thread for current sensor
    threads = []
    for sensor in AHRS:
        t = threading.Thread(target=send_data, args=(sensor, sensor.get_topic()), daemon=True)
        threads.append(t)
        t.start()

    try:
        while any(t.is_alive() for t in threads):
            time.sleep(1)
    except KeyboardInterrupt:
        print("Stopping data transmission...")


def send_Lidar_data(lidar, mqtt_client):

    def send_data(queue, topic):
      
        while True:
            try:
                data = queue.get_nowait()  # Load data without block
                mqtt_client.client.publish(topic, data)
                #print(f"Sent data to topic {topic}: {data}")

            except Exception as e:
                print(f"send lidar data exception: {e}")
                return
            time.sleep(0.5) 

    queue_topic_mapping = {
        lidar.imu_queue: "Lidar/imu",
        lidar.cloud_queue: "Lidar/cloud",
        lidar.dirty_queue: "Lidar/dirty",
    }

    threads = []
    for queue, topic in queue_topic_mapping.items():
        t = threading.Thread(target=send_data, args=(queue, topic), daemon=True)
        threads.append(t)
        t.start()

    try:
        while any(t.is_alive() for t in threads):
            time.sleep(1)
    except KeyboardInterrupt:
        print("Stopping data transmission...")
