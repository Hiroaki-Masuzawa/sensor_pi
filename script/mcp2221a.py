from i2cabst import I2CAbst
from PyMCP2221A import PyMCP2221A
import time
import os
os.environ["MCP2221_I2C_SLEEP"] = "0.05"
class PyMCP2221A_I2C(I2CAbst):
    def __init__(self):
        self.mcp2221 = PyMCP2221A.PyMCP2221A()
        self.mcp2221.I2C_Init()
    def write_i2c_block_data(self, addr, cmd, data):
        return self.mcp2221.I2C_Write(addr,[cmd]+data)
    def read_i2c_block_data(self, addr, cmd, length):
        self.mcp2221.I2C_Write_No_Stop(addr,[cmd])
        ret = self.mcp2221.I2C_Read_Repeated(addr, length)
        return ret
    def write_byte(self, addr, data):
        return self.mcp2221.I2C_Write(addr, [data])
    def write_byte_data(self, addr, cmd, data):
        return self.mcp2221.I2C_Write(addr, [cmd, data])