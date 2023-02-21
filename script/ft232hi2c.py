from os import environ
from pyftdi.i2c import I2cController
from i2cabst import I2CAbst

class FT232HI2C(I2CAbst):
    def __init__(self, bus_id):
        self.i2c = I2cController()
        url = environ.get('FTDI_DEVICE', 'ftdi://ftdi:232h/{}'.format(bus_id))
        self.i2c.configure(url, frequency=100000)

    def write_i2c_block_data(self, addr, cmd, data):
        port = self.i2c.get_port(addr)
        return port.write_to(cmd, data)
    def read_i2c_block_data(self, addr, cmd, length):
        port = self.i2c.get_port(addr)
        return port.read_from(cmd, length)

    def write_byte(self, addr, data):
        port = self.i2c.get_port(addr)
        if type(data) is int:
            data = [data]
        return port.write(data)

    def write_byte_data(self, addr, cmd, data):
        if type(data) is int:
            data = [data]
        port = self.i2c.get_port(addr)
        # print("write_byte_data", addr, cmd, data, type(data), type(data) is int)
        return port.write_to(cmd, data)