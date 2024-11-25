import board
import busio
from adafruit_tca9548a import TCA9548A

i2c = busio.I2C(board.SCL, board.SDA)
tca = TCA9548A(i2c)

for channel in range(8):
    if tca[channel].try_lock():
        print(f"Scanning channel {channel}...")
        addresses = tca[channel].scan()
        if addresses:
            print(f"Found devices at addresses: {[hex(address) for address in addresses]}")
        else:
            print("No devices found on this channel.")
        tca[channel].unlock()