import statistics
import struct
from adafruit_vl6180x import VL6180X
from .Mux_i2c import select_channel
import queue
import sys
sys.path.append('../')

class Vl6180x:
    sensors = {}
    channels = [1, 2, 3]
    distances = {}

    def __init__(self, i2c):
        self.i2c = i2c
        self.sensors = {}
        self.channels = [1, 2, 3]
        self.distances = {}
        self.data_queue = queue.Queue() 
        self.topic = "AHRS/vl6180x"

        for channel in self.channels:
            select_channel(self.i2c, channel)
            try:
                sensor = VL6180X(i2c)
                sensor.range
                self.sensors[channel] = sensor
                print(f"VL6180X on channel {channel}")
            except (ValueError, OSError, RuntimeError):
                print(f"No VL6180X on channel {channel}")
        
    def get_topic(self):
        return self.topic
    
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

    def save_to_queue(self):
        for channel, distances in self.distances.items():
            distances = [d for d in distances if d is not None]

            if distances:
                mean_distance = statistics.mean(distances)
                data_list = [channel, round(mean_distance, 2)]

                # Convert to bytearray
                data_to_queue = bytearray(struct.pack('If', data_list[0], data_list[1]))
                self.data_queue.put(data_to_queue)  
                #print(f"Queued data: {data_to_queue}")

        self.distances.clear()

    def get_data_from_queue(self):
        if not self.data_queue.empty():
            return self.data_queue.get()
        else:
            #print("\nEmpty queue VL6180X")
            return None

