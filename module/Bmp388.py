from adafruit_bmp3xx import BMP3XX_I2C
from .Mux_i2c import select_channel

class Bmp388:
     channel = 0
  
     def __init__(self, i_i2c):
          self.i2c = i_i2c
          open("/home/karol/Desktop/repos/SLAM/data/current/bmp388.txt", "w").close()
          open("/home/karol/Desktop/repos/SLAM/data/package/bmp388.txt", "w").close()
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
            file_path1 = "/home/karol/Desktop/repos/SLAM/data/current/bmp388.txt"
            file_path2 = "/home/karol/Desktop/repos/SLAM/data/package/bmp388.txt"
    
            try:
                temperature = self.sensor.temperature
                pressure = self.sensor.pressure
                data_to_write = f"BMP388 Temperature: {temperature:.2f} C, Pressure: {pressure:.2f} hPa\n\n"

                with open(file_path1, "w") as bmp388_file1, open(file_path2, "a") as bmp388_file2:
                    bmp388_file1.write(data_to_write)
                    bmp388_file2.write(data_to_write)
            except Exception as e:
                print(f"Error while saving BMP388 data: {e}")
