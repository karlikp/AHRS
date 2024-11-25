from adafruit_bmp3xx import BMP3XX_I2C
from .Mux_i2c import select_channel

class Bmp388:
     channel = 0
  
     def __init__(self, i_i2c):
          self.i2c = i_i2c
          select_channel(self.i2c, self.channel)
          
          try:
            self.sensor = BMP3XX_I2C(self.i2c, address=0x76)
            self.sensor.oversampling = 16
            self.sensor.iir_filter = 3
            print("BMP388 detected on channel 0")
          except Exception as e:
            print("Error initializing BMP388:", e)

     def save_to_file(self):
         if self.sensor:
            select_channel(self.i2c, self.channel)
            with open("data/bmp388.txt", "w") as bmp388_file:
                try:
                    temperature = self.sensor.temperature
                    pressure = self.sensor.pressure
                    bmp388_file.write(f"BMP388 Temperature: {temperature:.2f} C, Pressure: {pressure:.2f} hPa\n\n")  
                except RuntimeError:
                    print("BMP388 Error reading data")
