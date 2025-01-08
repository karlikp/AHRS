import sys
import time
sys.path.append("/home/karol/Desktop/repos/SLAM/module")
from module.Mqtt_module import Mqtt
import threading
import time



def send_AHRS_data(AHRS, mqtt_client):
    
    print(f"Sending AHRS")
    
    def send_data(sensor, topic):
        while True:
            try:
                data = sensor.get_data_from_queue()
                
                # check emptines
                if data:
                    mqtt_client.client.publish(topic, data)
                    print(f"Sent data: {data}")
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
    print(f"Sending Lidar")

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

def lidar_reading(lidar):

    print("Starting lidar reading thread")
    try:
        lidar.check_init()
        lidar.check_dirty()
        lidar.parsing_data()
    except Exception as e:
        print(f"Error in lidar_reading: {e}")

def interpolate_missing_angles(points):
    """
    Uzupełnia brakujące kąty w danych za pomocą interpolacji kwadratowej.
    
    :param points: Lista punktów w formacie (distance_mm, angle) z unikalnymi kątami
    :return: Lista punktów uzupełnionych o brakujące kąty
    """
    
    # Rozpakuj dane
    distances, angles = zip(*points)
    distances = np.array(distances)
    angles = np.array(angles)
    
    # Pełny zakres kątów 0-360
    full_angles = np.arange(0, 360, 1)
    
    # Funkcja interpolacji kwadratowej
    def quadratic_interpolation(x, x0, x1, y0, y1):
        """Interpolate using a quadratic function between two points."""
        # Przygotowanie współczynników dla funkcji kwadratowej y = ax^2 + bx + c
        a = (y1 - y0) / ((x1 - x0)**2)
        b = -2 * a * x0
        c = y0 + a * x0**2
        return a * x**2 + b * x + c
    
    # Przechowujemy interpolowane odległości
    interpolated_distances = []
    
    # Wykonaj interpolację dla każdego pełnego kąta
    for angle in full_angles:
        # Znajdź dwa najbliższe punkty (zdefiniowane przez angle) do wykonania interpolacji
        if angle <= angles[0]:
            interpolated_distances.append(distances[0])
        elif angle >= angles[-1]:
            interpolated_distances.append(distances[-1])
        else:
            # Znajdź dwa punkty sąsiednie, które otaczają nasz kąt
            for i in range(len(angles) - 1):
                if angles[i] <= angle < angles[i + 1]:
                    x0, x1 = angles[i], angles[i + 1]
                    y0, y1 = distances[i], distances[i + 1]
                    # Interpoluj za pomocą funkcji kwadratowej
                    interpolated_distance = quadratic_interpolation(angle, x0, x1, y0, y1)
                    interpolated_distances.append(interpolated_distance)
                    break
    
    # Zwróć dane w formacie (distance, angle)
    interpolated_points = [(int(distance), int(angle)) for distance, angle in zip(interpolated_distances, full_angles)]
    return interpolated_points

def filter_distances(points):
        return [point[0] for point in points]