import time

class ADS1015:
    
    class ADS1015_DATA_ADDRESS:
        REG_CONV = 0x00
        REG_CFG = 0x01
    
    class ADS1015_CFG_DATA:
        SINGLE_SHOT = 0x0100
        COMP_QUE_ONCE = 0x0000
        COMP_QUE_TWICE = 0x0001
        COMP_QUE_FOUR_TIMES = 0x0002
        COMP_QUE_DISABLE = 0x0003
        START_SINGLE_CONVERSION = 0x8000
    
    class ADS1015_PGA:
        PGA_6_144V = 6144
        PGA_4_096V = 4096
        PGA_2_048V = 2048
        PGA_1_024V = 1024
        PGA_0_512V = 512
        PGA_0_256V = 256
    
    class ADS1015_MUX:
        AIN0_AIN1 = 0x0000
        AIN0_AIN3 = 0x1000
        AIN1_AIN3 = 0x2000
        AIN2_AIN3 = 0x3000
        AIN0_GND  = 0x4000
        AIN1_GND  = 0x5000
        AIN2_GND  = 0x6000
        AIN3_GND  = 0x7000
    
    def get_sps_data(self, sps):
        if sps == 128:
            return 0x0000
        elif sps == 250:
            return 0x0020
        elif sps == 490:
            return 0x0040
        elif sps == 920:
            return 0x0060
        elif sps == 1600:
            return 0x0080
        elif sps == 2400:
            return 0x00A0
        elif sps == 3300:
            return 0x00C0
        
    def get_channel_data(self, ch):
        if ch == 0:
            return self.ADS1015_MUX.AIN0_GND
        elif ch == 1:
            return self.ADS1015_MUX.AIN1_GND
        elif ch == 2:
            return self.ADS1015_MUX.AIN2_GND
        elif ch == 3:
            return self.ADS1015_MUX.AIN3_GND
    
    def get_pg_data(self, pg):
        if pg == 6144:
            return 0x0000
        elif pg == 4096:
            return 0x0200
        elif pg == 2048:
            return 0x0400
        elif pg == 1024:
            return 0x0600
        elif pg == 512:
            return 0x0800
        elif pg == 256:
            return 0x0A00
         
    def __init__(self, i2c, address):
        self.i2c = i2c
        self.address = address
        self.programmable_gain = self.ADS1015_PGA.PGA_6_144V
        self.samples_per_second = 128
    
    def busy(self):
        data = self.i2c.read_i2c_block_data(self.address, self.ADS1015_DATA_ADDRESS.REG_CFG, 2)
        status = (data[0] << 8) | data[1]
        return (status & (1 << 15)) == 0
    
    def read_se_adc(self, mux):

        config = self.ADS1015_CFG_DATA.COMP_QUE_DISABLE | self.ADS1015_CFG_DATA.SINGLE_SHOT

        config |= self.get_sps_data(self.samples_per_second)
        config |= mux
        config |= self.get_pg_data(self.programmable_gain)

        config |= self.ADS1015_CFG_DATA.START_SINGLE_CONVERSION
        
        self.i2c.write_i2c_block_data(self.address, self.ADS1015_DATA_ADDRESS.REG_CFG, [(config >> 8) & 0xFF, config & 0xFF])
        
        while self.busy():
            time.sleep(0.001)
        
        data = self.i2c.read_i2c_block_data(self.address, self.ADS1015_DATA_ADDRESS.REG_CONV, 2)
        value = (data[0] << 4) | (data[1] >> 4)
        if value & 0x800:
            value -= 1 << 12
        value /= 2047.0
        value *= float(self.programmable_gain)  
        value /= 1000.0

        return value
