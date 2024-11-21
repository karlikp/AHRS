import statistics
from adafruit_vl53l1x import VL53L1X
from .Mux_i2c import select_channel


class Vl53l1x:
    sensors = {}
    channels = [4, 5, 6]
    distances = {}

    def __init__(self, i2c):
            self.i2c = i2c
            for channel in self.channels:
                select_channel(self.i2c, channel)
                try:
                    sensor = VL53L1X(i2c)
                    sensor.distance
                    self.sensors[channel] = sensor
                    print(f"VL53L1X on channel {channel}")
                except (ValueError, OSError, RuntimeError):
                    print(f"No VL53L1X on channel {channel}")

    def collect_data(self):
        for channel, sensor in self.sensors.items():
                select_channel(self.i2c, channel)
                try:
                    distance = sensor.distance
                    if distance is not None:
                        distance = distance * 10  # Convert to mm
                    if channel not in self.distances:
                        self.distances[channel] = []
                    self.distances[channel].append(distance)
                except RuntimeError:
                    print(f"Channel {channel}, VL53L1X Error reading distance")

    def save_to_file(self):
        with open("/home/karol/Desktop/repos/SLAM/data/vl53l1x.txt", "w") as distance_file:
            for channel, distances in self.distances.items():
                distances = [d for d in distances if d is not None]  # Remove None values
                if distances:
                    mean_distance = statistics.mean(distances)
                    distance_file.write(f"Mean VL53L1X Channel {channel} Distance: {mean_distance:.2f} mm\n")
                    distances.clear()
        