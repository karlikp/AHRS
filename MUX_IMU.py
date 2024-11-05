import time
import board
import busio
from adafruit_bmp3xx import BMP3XX_I2C
from smbus2 import SMBus

MUX_ADDRESS = 0x70

def select_channel(i2c, channel):
    print(f"Set channel {channel}")
    if 0 <= channel <= 7:  
        i2c.writeto(MUX_ADDRESS, bytes([1 << channel]))
    else:
        raise ValueError("Channel must be between 0 and 7")


def initialize_bmp388(i2c):
    bmp388 = BMP3XX_I2C(i2c, address=0x76) # creating an instance BMP388 
    bmp388.oversampling = 16  
    bmp388.iir_filter = 3  
    return bmp388

def read_bmx160(bus):
    bmx160_address = 0x68 
    bus.write_byte_data(bmx160_address, 0x7E, 0xB6)  # sensor reset
    time.sleep(0.1)  

    # sensor setup
    bus.write_byte_data(bmx160_address, 0x7C, 0x00)  # Set the normal mood
    bus.write_byte_data(bmx160_address, 0x7D, 0x06)  # Set the resolution
    time.sleep(0.1) 

    data = bus.read_i2c_block_data(bmx160_address, 0x12, 6)
    accel_x = (data[1] << 8) | data[0]
    accel_y = (data[3] << 8) | data[2]
    accel_z = (data[5] << 8) | data[4]

    return accel_x, accel_y, accel_z


i2c = busio.I2C(board.SCL, board.SDA)
bus = SMBus(1)  # 1 is recomended for I2C

select_channel(i2c, 0) 
bmp388_sensor = initialize_bmp388(i2c)

try:
    while True:
        temperature = bmp388_sensor.temperature
        pressure = bmp388_sensor.pressure
        
        bmx160_data = read_bmx160(bus)
        
        print(f"BMX160 Data: Accel X: {bmx160_data[0]}, Accel Y: {bmx160_data[1]}, Accel Z: {bmx160_data[2]}")
        print(f"BMP388 Temperature: {temperature:.2f} C, Pressure: {pressure:.2f} hPa")
        
        time.sleep(1)
        
except KeyboardInterrupt:
    print("Program interrupted")


finally:
    bus.close() 
