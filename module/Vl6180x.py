import statistics
from adafruit_vl6180x import VL6180X
from .Mux_i2c import select_channel
import sys
sys.path.append('../')

class Vl6180x:
    sensors = {}
    channels = [1, 2, 3]
    distances = {}

    def __init__(self, i2c):
        self.i2c = i2c

        open("/home/karol/Desktop/repos/SLAM/data/current/distances.txt", "w").close()
        open("/home/karol/Desktop/repos/SLAM/data/package/distances.txt", "w").close()

        for channel in self.channels:
            select_channel(self.i2c, channel)
            try:
                sensor = VL6180X(i2c)
                sensor.range
                self.sensors[channel] = sensor
                print(f"VL6180X on channel {channel}")
            except (ValueError, OSError, RuntimeError):
                print(f"No VL6180X on channel {channel}")
        
    
    def collect_data(self):
        for channel, sensor in self.sensors.items():
            select_channel(self.i2c, channel)
            try:
                distance = sensor.range
                if channel not in self.distances:
                    self.distances[channel] = []
                self.distances[channel].append(distance)
            except RuntimeError:
                print(f"Channel {channel}, VL6180X Error reading distance")

    def save_to_file(self):
    
        file_path1 = "/home/karol/Desktop/repos/SLAM/data/current/distances.txt"
        file_path2 = "/home/karol/Desktop/repos/SLAM/data/package/distances.txt"

        with open(file_path1, "w") as distance_file1, open(file_path2, "a") as distance_file2:
            for channel, distances in self.distances.items():
                
                distances = [d for d in distances if d is not None]
                
                if distances:
                    mean_distance = statistics.mean(distances)
                
                    distance_file1.write(f"Channel {channel} Distance: {mean_distance:.2f} mm\n")
                    distance_file2.write(f"Channel {channel} Distance: {mean_distance:.2f} mm\n")

        self.distances.clear()

