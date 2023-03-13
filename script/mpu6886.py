import numpy as np
import time

class MPU6886:
    class Ascale:
        AFS_2G = 0
        AFS_4G = 1
        AFS_8G = 2
        AFS_16G = 3
    
    class Gscale:
        GFS_250DPS = 0
        GFS_500DPS = 1
        GFS_1000DPS = 2
        GFS_2000DPS = 3
    
    class COMMAND:
        IMU_6886_WHOAMI            = 0x75
        IMU_6886_ACCEL_INTEL_CTRL  = 0x69
        IMU_6886_SMPLRT_DIV        = 0x19
        IMU_6886_INT_PIN_CFG       = 0x37
        IMU_6886_INT_ENABLE        = 0x38
        IMU_6886_ACCEL_XOUT_H      = 0x3B
        IMU_6886_ACCEL_XOUT_L      = 0x3C
        IMU_6886_ACCEL_YOUT_H      = 0x3D
        IMU_6886_ACCEL_YOUT_L      = 0x3E
        IMU_6886_ACCEL_ZOUT_H      = 0x3F
        IMU_6886_ACCEL_ZOUT_L      = 0x40
        IMU_6886_TEMP_OUT_H        = 0x41
        IMU_6886_TEMP_OUT_L        = 0x42
        IMU_6886_GYRO_XOUT_H       = 0x43
        IMU_6886_GYRO_XOUT_L       = 0x44
        IMU_6886_GYRO_YOUT_H       = 0x45
        IMU_6886_GYRO_YOUT_L       = 0x46
        IMU_6886_GYRO_ZOUT_H       = 0x47
        IMU_6886_GYRO_ZOUT_L       = 0x48
        IMU_6886_USER_CTRL         = 0x6A
        IMU_6886_PWR_MGMT_1        = 0x6B
        IMU_6886_PWR_MGMT_2        = 0x6C
        IMU_6886_CONFIG            = 0x1A
        IMU_6886_GYRO_CONFIG       = 0x1B
        IMU_6886_ACCEL_CONFIG      = 0x1C
        IMU_6886_ACCEL_CONFIG2     = 0x1D
        IMU_6886_FIFO_EN           = 0x23
        IMU_6886_FIFO_ENABLE       = 0x23
        IMU_6886_FIFO_COUNT        = 0x72
        IMU_6886_FIFO_R_W          = 0x74
        IMU_6886_GYRO_OFFSET       = 0x13
        
    def __init__(self, i2c, address=0x68):
        self.i2c = i2c
        self.address = address
        self.Gyscale = self.Gscale.GFS_2000DPS
        self.Acscale = self.Ascale.AFS_8G
    
    def dev_init(self):
        tempdata = self.i2c.read_i2c_block_data(self.address, self.COMMAND.IMU_6886_WHOAMI, 1)
        imuId = tempdata[0]
        time.sleep(0.001)
        
        regdata = [0x00]
        self.i2c.write_i2c_block_data(self.address, self.COMMAND.IMU_6886_PWR_MGMT_1, regdata)
        time.sleep(0.01)
                
        regdata = [0x01<<7]
        self.i2c.write_i2c_block_data(self.address, self.COMMAND.IMU_6886_PWR_MGMT_1, regdata)
        time.sleep(0.01)
        
        regdata = [0x00]
        self.i2c.write_i2c_block_data(self.address, self.COMMAND.IMU_6886_PWR_MGMT_1, regdata)
        time.sleep(0.01)
        
        regdata = [0x10]
        self.i2c.write_i2c_block_data(self.address, self.COMMAND.IMU_6886_ACCEL_CONFIG, regdata)
        time.sleep(0.001)
        
        regdata = [0x18]
        self.i2c.write_i2c_block_data(self.address, self.COMMAND.IMU_6886_GYRO_CONFIG, regdata)
        time.sleep(0.001)
        
        regdata = [0x01]
        self.i2c.write_i2c_block_data(self.address, self.COMMAND.IMU_6886_CONFIG, regdata)
        time.sleep(0.001)
        
        regdata = [0x01]
        self.i2c.write_i2c_block_data(self.address, self.COMMAND.IMU_6886_SMPLRT_DIV, regdata)
        time.sleep(0.001)
        
        regdata = [0x00]
        self.i2c.write_i2c_block_data(self.address, self.COMMAND.IMU_6886_INT_ENABLE, regdata)
        time.sleep(0.001)
        
        regdata = [0x00]
        self.i2c.write_i2c_block_data(self.address, self.COMMAND.IMU_6886_ACCEL_CONFIG2, regdata)
        time.sleep(0.001)
        
        regdata = [0x00]
        self.i2c.write_i2c_block_data(self.address, self.COMMAND.IMU_6886_USER_CTRL, regdata)
        time.sleep(0.001)
        
        regdata = [0x00]
        self.i2c.write_i2c_block_data(self.address, self.COMMAND.IMU_6886_FIFO_EN, regdata)
        time.sleep(0.001)
        
        regdata = [0x22]
        self.i2c.write_i2c_block_data(self.address, self.COMMAND.IMU_6886_INT_PIN_CFG, regdata)
        time.sleep(0.001)
        
        regdata = [0x01]
        self.i2c.write_i2c_block_data(self.address, self.COMMAND.IMU_6886_INT_ENABLE, regdata)
        time.sleep(0.01)
        
        self.setGyroFsr(self.Gyscale)
        self.setAccelFsr(self.Acscale)
        
    def setGyroFsr(self, scale):
        regdata = [scale<<3]
        self.i2c.write_i2c_block_data(self.address, self.COMMAND.IMU_6886_GYRO_CONFIG, regdata)
        time.sleep(0.01)
        self.updateGres()
        
    def updateGres(self):
        if self.Gyscale == self.Gscale.GFS_250DPS:
            self.gRes = 250.0/32768.0
        elif self.Gyscale == self.Gscale.GFS_500DPS:
            self.gRes = 500.0/32768.0
        elif self.Gyscale == self.Gscale.GFS_1000DPS:
            self.gRes = 1000.0/32768.0
        elif self.Gyscale == self.Gscale.GFS_2000DPS:
            self.gRes = 2000.0/32768.0
        
    def setAccelFsr(self, scale):
        regdata = [scale<<3]
        self.i2c.write_i2c_block_data(self.address, self.COMMAND.IMU_6886_ACCEL_CONFIG, regdata)
        time.sleep(0.01)
        self.updateAres()
        
    def updateAres(self):
        if self.Acscale == self.Ascale.AFS_2G:
            self.aRes = 2.0/32768.0
        elif self.Acscale == self.Ascale.AFS_4G:
            self.aRes = 4.0/32768.0
        elif self.Acscale == self.Ascale.AFS_8G:
            self.aRes = 8.0/32768.0
        elif self.Acscale == self.Ascale.AFS_16G:
            self.aRes = 16.0/32768.0
        
        
    def getGyroAdc(self):
        buf = self.i2c.read_i2c_block_data(self.address, self.COMMAND.IMU_6886_GYRO_XOUT_H, 6)
        
        gx = buf[0] << 8 | buf[1]
        gy = buf[2] << 8 | buf[3]
        gz = buf[4] << 8 | buf[5]
        ret = [gx, gy, gz]
        return np.array(ret, dtype=np.int16)
        
    def getGyroData(self):
        dat = self.getGyroAdc()
        
        return np.array([dat[0]*self.gRes, dat[1]*self.gRes, dat[2]*self.gRes])
        
    def getAccelAdc(self):
        buf = self.i2c.read_i2c_block_data(self.address, self.COMMAND.IMU_6886_ACCEL_XOUT_H, 6)
        
        ax = buf[0] << 8 | buf[1]
        ay = buf[2] << 8 | buf[3]
        az = buf[4] << 8 | buf[5]
        ret = [ax, ay, az]
        return np.array(ret, dtype=np.int16)
        
    def getAccelData(self):
        dat = self.getAccelAdc()
        return np.array([dat[0]*self.aRes, dat[1]*self.aRes, dat[2]*self.aRes])
    
    def getTempAdc(self):
        buf = self.i2c.read_i2c_block_data(self.address, self.COMMAND.IMU_6886_TEMP_OUT_H, 2)
        ret = buf[0] << 8 | buf[1]
        return np.array(ret, dtype=np.int16)
        
    def getTempData(self):
        dat = self.getTempAdc()
        return float(dat)/ 326.8 + 25.0


# https://github.com/xioTechnologies/Fusion
if __name__ == '__main__':
    from mcp2221a import PyMCP2221A_I2C
    import imufusion

    i2c = PyMCP2221A_I2C()
    imu = MPU6886(i2c)

    imu.dev_init()
    ahrs = imufusion.Ahrs()
    last_time = None
    while True:
        g_data = imu.getGyroData()
        a_data = imu.getAccelData()
        t_data = imu.getTempData()

        d_time = 0.01
        n_time = time.time()
        if last_time is not None:
            d_time = n_time-last_time
            print(d_time)
        print(type(g_data), type(a_data), type(d_time))
        ahrs.update_no_magnetometer(g_data, a_data, d_time)
        last_time = n_time

        print("gdata : {}\nadata : {}\ntdata : {}\n".format(g_data, a_data, t_data))
        print(ahrs.quaternion.w,ahrs.quaternion.x,ahrs.quaternion.y,ahrs.quaternion.z)
        time.sleep(0.01)