from adafruit_bmp3xx import BMP3XX_I2C
from .i2c_mux import select_channel

def initialize_bmp388(i2c):
    select_channel(i2c, 0) #BMP388 on channel 0
    try:
        bmp388 = BMP3XX_I2C(i2c, address=0x76)
        bmp388.oversampling = 16
        bmp388.iir_filter = 3
        print("BMP388 detected on channel 0")
        return bmp388
    except Exception as e:
        print("Error initializing BMP388:", e)
        return None