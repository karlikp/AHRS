MUX_ADDRESS = 0x70

def select_channel(i2c, channel):
    if 0 <= channel <= 7:
        i2c.writeto(MUX_ADDRESS, bytes([1 << channel]))
    else:
        raise ValueError("Channel must be between 0 and 7")