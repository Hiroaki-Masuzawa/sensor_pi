from i2cabst import I2CAbst
import smbus2

class SMBus2I2C(I2CAbst):
    def __init__(self, bus_id):
        self.i2c = smbus2.SMBus(bus_id)
    def write_i2c_block_data(self, addr, cmd, data):
        return self.i2c.write_i2c_block_data(addr, cmd, data)
    def read_i2c_block_data(self, addr, cmd, length):
        return self.i2c.read_i2c_block_data(addr, cmd, length)
    def write_byte(self, addr, data):
        return self.i2c.write_byte(addr, data)
    def write_byte_data(self, addr, cmd, data):
        return self.i2c.write_byte_data(addr, cmd, data)