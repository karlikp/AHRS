import queue
import struct
import os
import sys

# Adding dir 'utils' to sys.path based on main project dir
sys.path.append(os.path.join(os.path.dirname(__file__), 'utils'))
from utils import select_channel

from adafruit_bmp3xx import BMP3XX_I2C

class Bmp388:
  channel = 0
  
  def __init__(self, i_i2c):
      self.i2c = i_i2c
      self.data_queue = queue.Queue()  
      select_channel(self.i2c, self.channel)
      
      try:
        self.sensor = BMP3XX_I2C(self.i2c, address=0x76)
        self.sensor.oversampling = 16
        self.sensor.iir_filter = 3
        self.topic = "AHRS/bmp388"
        print("BMP388 detected on channel 0")
      except Exception as e:
        self.sensor = None
        print("Error initializing BMP388:", e)

  def get_topic(self):
     return self.topic

  def save_to_queue(self):
    if self.sensor:
        select_channel(self.i2c, self.channel)

        try:
            temperature = self.sensor.temperature
            pressure = self.sensor.pressure

            #`bytearray`(format: 2 floaty)
            packed_data = bytearray(struct.pack('2f', round(temperature, 2), round(pressure, 2)))

            self.data_queue.put(packed_data)
        except Exception as e:
            print(f"Error while saving BMP388 data to queue: {e}")


  def get_data_from_queue(self):
  
    if not self.data_queue.empty():
        return self.data_queue.get()
    else:
        print("\nEmpty queue bmp388")
        return None
