import statistics
import queue
import struct
from adafruit_vl53l1x import VL53L1X
from .Mux_i2c import select_channel



class Vl53l1x:
    data_queue = queue.Queue()

    def __init__(self, i2c):
        self.i2c = i2c
        self.sensors = {}
        self.channels = {4, 5, 6}
        self.distances = {}
        self.topic = "AHRS/vl53l1x"

        for channel in self.channels:
            select_channel(self.i2c, channel)
            try:
                sensor = VL53L1X(self.i2c)
                sensor.start_ranging() 
                # Try read range to check sensor working
                self.sensors[channel] = sensor
                print(f"VL53L1X on channel {channel}")
            except (ValueError, OSError, RuntimeError):
                print(f"No VL53L1X on channel {channel}")

    def get_topic(self):
        return self.topic
    
    def collect_data(self):
        for channel, sensor in self.sensors.items():
                select_channel(self.i2c, channel)
                try:
                    distance = sensor.distance
                    if distance is not None:
                        distance = distance * 10  # Convert to mm
                    else:
                        distance = 3500
                    if channel not in self.distances:
                        self.distances[channel] = []
                    self.distances[channel].append(distance)
                except RuntimeError:
                    print(f"Channel {channel}, VL53L1X Error reading distance")

    def save_to_queue(self):
        for channel, distances in self.distances.items():
            distances = [d for d in distances if d is not None]

            if distances:        
                mean_distance = statistics.mean(distances)

                # Prepare bytearray
                # Format 'If' mean: I - unsigned int, f - float
                data_to_queue = bytearray(struct.pack('If', channel, round(mean_distance, 2)))

                self.data_queue.put(data_to_queue)
                #print(f"Queued bytearray data: {data_to_queue}")

        self.distances.clear()

    def get_data_from_queue(self):
        if not self.data_queue.empty():
            return self.data_queue.get()
        else:
            #print("\nEmpty queue VL53L1X")
            return None