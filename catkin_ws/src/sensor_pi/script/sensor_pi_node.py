#!/usr/bin/env python
import rospy
import smbus
import time
from std_msgs.msg import Float64

class I2CSensorPublisherNodeAbst(object):
    def __init__(self, i2c, address):
        self.i2c = i2c
        self.address = address
    def loop(self):
        None

class TOFPublisher(I2CSensorPublisherNodeAbst):
    def __init__(self, i2c, address):
        super(TOFPublisher, self).__init__(i2c, address)
        self.pub = rospy.Publisher('~tof_distance', Float64, queue_size=1)
        self.VL53L0X_REG_SYSRANGE_START = 0x00
        self.VL53L0X_REG_RESULT_RANGE_STATUS = 0x14
        self.pub = rospy.Publisher('~tof_distance', Float64, queue_size=1)
    def loop(self):
        self.i2c.write_i2c_block_data(self.address, self.VL53L0X_REG_SYSRANGE_START, [0x01])
        cnt = 0
        while cnt < 100:
            time.sleep(0.01)
            val = self.i2c.read_i2c_block_data(self.address, self.VL53L0X_REG_RESULT_RANGE_STATUS, 1)
            if val[0] & 0x01 != 0x00:
                break
            cnt+=1
        val = self.i2c.read_i2c_block_data(self.address, self.VL53L0X_REG_RESULT_RANGE_STATUS, 14)
        if cnt < 100:
            #acnt = val[6] * 2**8 + val[7] 
            #scnt = val[8] * 2**8 + val[9] 
            distance = float(val[10] * 2**8 + val[11])
            self.pub.publish(distance)




class UltraSonicPublisher(I2CSensorPublisherNodeAbst):
    def __init__(self, i2c, address):
        super(UltraSonicPublisher, self).__init__(i2c, address)
        self.pub = rospy.Publisher('~ultrasonic_distance', Float64, queue_size=1)
    def loop(self):
        self.i2c.write_i2c_block_data(self.address, 0x01,[])
        time.sleep(0.12)
        data = self.i2c.read_i2c_block_data(self.address, 0x00, 3)
        distance =  float(data[0] * (2**16) + data[1] * (2**8)+ data[2])/ 1000
        self.pub.publish(distance)


class SensorPublisher():
    def __init__(self, bus_id=1):
        rospy.init_node('sensor_pi_node', anonymous=True)
        self.i2c = smbus.SMBus(bus_id)
        self.node_list = list()
        use_ultrasonic = rospy.get_param("~use_ultrasonic")
        use_tof = rospy.get_param("~use_tof")
        if use_ultrasonic:
            self.node_list.append(UltraSonicPublisher(self.i2c, 0x57))
        if use_tof:
            self.node_list.append(TOFPublisher(self.i2c, 0x29))
    def loop(self):
        for node in self.node_list:
            node.loop()
        if len(self.node_list) == 0:
            time.sleep(0.1)


if __name__ == '__main__':
    sensor_pub = SensorPublisher()
    while not rospy.is_shutdown():
        sensor_pub.loop()
