import smbus
import time

# https://stackoverflow.com/questions/41335921/how-to-read-from-multiplexer-with-python-i2c-tca9548a
TCA9548A_address = 0x70

class TCA9548A:
    TCA9548A_CH ={
            0 : 0b00000001,
        1 : 0b00000010,
        2 : 0b00000100,
        3 : 0b00001000,
        4 : 0b00010000,
        5 : 0b00100000,
        6 : 0b01000000,
        7 : 0b10000000,
        }
    def __init__(self, bus, address):
        self.bus = bus
        self.address = address
    def set_ch(self, ch):
        self.bus.write_byte(self.address, self.TCA9548A_CH[ch])
        time.sleep(0.1)

