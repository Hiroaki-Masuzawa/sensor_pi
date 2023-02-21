class I2CAbst(object):
    def __init__(self):
        None
    def write_i2c_block_data(self, addr, cmd, data):
        assert False, "Not defined write_i2c_block_data"
    def read_i2c_block_data(self, addr, cmd, length):
        assert False, "Not defined read_i2c_block_data"
    def write_byte(self, addr, data):
        assert False, "Not defined write_byte"
    def write_byte_data(self, addr, cmd, data):
        assert False, "Not defined write_byte_data"