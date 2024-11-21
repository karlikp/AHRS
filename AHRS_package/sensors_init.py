from smbus2 import SMBus
from .Vl6180x import initialize_vl6180x
from .Vl53l1x import initialize_vl53l1x
#from .bmp388_module import initialize_bmp388
from .bmx160_module import initialize_bmx160

#bus = SMBus(1)

def initialize_all_sensors(i2c, bus):
    sensors_vl6180x = initialize_vl6180x(i2c, [1, 2, 3])  # VL61 on channels: 1, 2, 3
    sensors_vl53l1x =  initialize_vl53l1x(i2c, [4, 5, 6]) # VL53 on channels: 4, 5, 6
    #bmp388_sensor = initialize_bmp388(i2c)  
    bmx160_initialized = initialize_bmx160(bus) 
    return sensors_vl6180x, sensors_vl53l1x, bmx160_initialized