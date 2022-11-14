import smbus
import time


# https://github.com/Seeed-Studio/Grove_I2C_Color_Sensor_TCS3472/blob/master/Adafruit_TCS34725.h
# 

# i2c address
TCS34725_ADDRESS = 0x29


class TCS34725:
    TCS34725_COMMAND_BIT = 0x80
    class TCS34725_DATA_ADDRESS:
        # device address 
        TCS34725_ENABLE = 0x00
        TCS34725_C_DATA_L = 0x14
        TCS34725_R_DATA_L = 0x16
        TCS34725_CONTROL = 0x0F
        TCS34725_ATIME = 0x01

    class TCS34725_COMMAND_DATA:
        # data
        TCS34725_ENABLE_PON = 0x01
        TCS34725_ENABLE_AEN = 0x02

    class TCS34725_GAIN_DATA:
        TCS34725_GAIN_1X = 0x00
        TCS34725_GAIN_4X = 0x01
        TCS34725_GAIN_16X = 0x02
        TCS34725_GAIN_60X = 0x03
    class TCS34725_INTEGRATIONTIME:
        TCS34725_INTEGRATIONTIME_2_4MS = 0xFF
        TCS34725_INTEGRATIONTIME_24MS  = 0xF6
        TCS34725_INTEGRATIONTIME_50MS  = 0xEB
        TCS34725_INTEGRATIONTIME_101MS = 0xD5
        TCS34725_INTEGRATIONTIME_154MS = 0xC0
        TCS34725_INTEGRATIONTIME_700MS = 0x00

    def __init__(self, address=TCS34725_ADDRESS, bus=None):
        if bus is None:
            self.bus = smbus.SMBus(1)
        else :
            self.bus = bus
        self.address = address

    def setup(self):
        self.enable()
        self.set_gain(self.TCS34725_GAIN_DATA.TCS34725_GAIN_60X)
        self.set_integrationtime(self.TCS34725_INTEGRATIONTIME.TCS34725_INTEGRATIONTIME_154MS)

    def set_gain(self, gain):
        data_addr = self.TCS34725_DATA_ADDRESS.TCS34725_CONTROL | self.TCS34725_COMMAND_BIT
        self.bus.write_byte_data(self.address, data_addr, gain)
        
    def set_integrationtime(self, integrationtime):
        self.integrationtime = integrationtime
        data_addr = self.TCS34725_DATA_ADDRESS.TCS34725_ATIME | self.TCS34725_COMMAND_BIT
        self.bus.write_byte_data(self.address, data_addr, self.integrationtime)
        self.sleep_integratioon_time()

    def sleep_integratioon_time(self):
        if self.integrationtime == self.TCS34725_INTEGRATIONTIME.TCS34725_INTEGRATIONTIME_2_4MS:
            time.sleep(0.003)
        elif self.integrationtime == self.TCS34725_INTEGRATIONTIME.TCS34725_INTEGRATIONTIME_24MS :
            time.sleep(0.024)
        elif self.integrationtime == self.TCS34725_INTEGRATIONTIME.TCS34725_INTEGRATIONTIME_50MS :
            time.sleep(0.050)
        elif self.integrationtime == self.TCS34725_INTEGRATIONTIME.TCS34725_INTEGRATIONTIME_101MS :
            time.sleep(0.101)
        elif self.integrationtime == self.TCS34725_INTEGRATIONTIME.TCS34725_INTEGRATIONTIME_154MS :
            time.sleep(0.154)
        elif self.integrationtime == self.TCS34725_INTEGRATIONTIME.TCS34725_INTEGRATIONTIME_700MS :
            time.sleep(0.700)
        return 

        
    def enable(self):
        data_addr = self.TCS34725_DATA_ADDRESS.TCS34725_ENABLE | self.TCS34725_COMMAND_BIT
        data_value = self.TCS34725_COMMAND_DATA.TCS34725_ENABLE_PON|self.TCS34725_COMMAND_DATA.TCS34725_ENABLE_AEN
        self.bus.write_byte_data(self.address, data_addr, data_value)

    def get_rawdata(self):
        data = self.bus.read_i2c_block_data(self.address, self.TCS34725_DATA_ADDRESS.TCS34725_C_DATA_L | self.TCS34725_COMMAND_BIT, 8)
        self.sleep_integratioon_time()
        clear_v = data[1] * 256 + data[0]
        red_v = data[3] * 256 + data[2]
        green_v = data[5] * 256 + data[4]
        blue_v = data[7] * 256 + data[6]
        return clear_v, red_v, green_v, blue_v

