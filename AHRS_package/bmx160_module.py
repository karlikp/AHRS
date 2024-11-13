from smbus2 import SMBus
import time

def initialize_bmx160(bus):
    bmx160_address = 0x68
    try:
        bus.write_byte_data(bmx160_address, 0x7E, 0xB6)  # Reset 
        time.sleep(0.1)
        bus.write_byte_data(bmx160_address, 0x7C, 0x00)  # normal mood
        bus.write_byte_data(bmx160_address, 0x7D, 0x06)  # Ustawienie rozdzielczości
        print("BMX160 detected on channel 0")
        return True
    except Exception as e:
        print("Error initializing BMX160:", e)
        return False

def read_bmx160(bus):
    bmx160_address = 0x68
    try:
        data = bus.read_i2c_block_data(bmx160_address, 0x12, 6)
        accel_x = (data[1] << 8) | data[0]
        accel_y = (data[3] << 8) | data[2]
        accel_z = (data[5] << 8) | data[4]
        return accel_x, accel_y, accel_z
    except OSError:
        return None