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
        for channel in self.channels:
            select_channel(self.i2c, channel)
            try:
                sensor = VL6180X(i2c)
                sensor.range
                self.sensors[channel] = sensor
                print(f"VL6180X on channel {channel}")
            except (ValueError, OSError, RuntimeError):
                print(f"No VL6180X on channel {channel}")
        
    # def initialize_vl6180x(i2c, channels):
    #     sensors_vl6180x = {}
    #     for channel in channels:
    #         select_channel(i2c, channel)
    #         try:
    #             sensor = VL6180X(i2c)
    #             sensor.range  # check if work
    #             sensors_vl6180x[channel] = sensor
    #             print(f"VL6180X detected on channel {channel}")
    #         except (ValueError, OSError, RuntimeError):
    #             print(f"No VL6180X on channel {channel}")
    #     return sensors_vl6180x
    

    # for channel, sensor in sensors_vl6180x.items():
    #             select_channel(i2c, channel)
    #             try:
    #                 distance = sensor.range
    #                 if channel not in vl6180x_distances:
    #                     vl6180x_distances[channel] = []
    #                 vl6180x_distances[channel].append(distance)
    #             except RuntimeError:
    #                 print(f"Channel {channel}, VL6180X Error reading distance")


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
        with open("/home/karol/Desktop/repos/SLAM/data/vl6180x.txt", "w") as distance_file:
            for channel, distances in self.distances.items():
                distances = [d for d in distances if d is not None]  # Remove None values
                if distances:
                    mean_distance = statistics.mean(distances)
                    distance_file.write(f"Mean Vl6180X Channel {channel} Distance: {mean_distance:.2f} mm\n")
                    distances.clear()
