#!/usr/bin/env python
import rospy
import smbus
import time
from std_msgs.msg import Float64, ColorRGBA
import json

class I2CSensorPublisherNodeAbst(object):
    def __init__(self, i2c, address):
        self.i2c = i2c
        self.address = address
    def setup(self):
        None
    def loop(self):
        None

class TOFPublisher(I2CSensorPublisherNodeAbst):
    def __init__(self, i2c, address, topic_name='tof_distance'):
        super(TOFPublisher, self).__init__(i2c, address)
        self.pub = rospy.Publisher('~{}'.format(topic_name), Float64, queue_size=1)
        self.VL53L0X_REG_SYSRANGE_START = 0x00
        self.VL53L0X_REG_RESULT_RANGE_STATUS = 0x14
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
    def __init__(self, i2c, address, topic_name = 'ultrasonic_distance'):
        super(UltraSonicPublisher, self).__init__(i2c, address)
        self.pub = rospy.Publisher('~{}'.format(topic_name), Float64, queue_size=1)
    def loop(self):
        self.i2c.write_i2c_block_data(self.address, 0x01,[])
        time.sleep(0.12)
        data = self.i2c.read_i2c_block_data(self.address, 0x00, 3)
        distance =  float(data[0] * (2**16) + data[1] * (2**8)+ data[2])/ 1000
        self.pub.publish(distance)



from tca9548 import TCA9548A, TCA9548A_address

class I2CHubPublisher(I2CSensorPublisherNodeAbst):
    def __init__(self, i2c, address, sensor_dict):
        super(I2CHubPublisher, self).__init__(i2c, address)
        self.sensors = sensor_dict
        self.tca = TCA9548A(i2c, address)
    def setup(self):
        for idx, sensor in self.sensors.items():
            self.tca.set_ch(idx)
            sensor.setup()
    def loop(self):
        for idx, sensor in self.sensors.items():
            self.tca.set_ch(idx)
            sensor.loop()

from tcs34725 import TCS34725, TCS34725_ADDRESS

class ColorSensorPublisher(I2CSensorPublisherNodeAbst):
    def __init__(self, i2c, address, topic_name = "color_value"):
        super(ColorSensorPublisher, self).__init__(i2c, address)
        self.sensor = TCS34725(address=address, bus=i2c)
        self.pub = rospy.Publisher('~{}'.format(topic_name), ColorRGBA, queue_size=1)
    def setup(self):
        self.sensor.setup()
    def loop(self):
        clear_v, red_v, green_v, blue_v = self.sensor.get_rawdata()
        color = ColorRGBA()
        max_count = max(1024*(256-self.sensor.integrationtime), 65535)
        color.r = red_v/max_count
        color.g = green_v/max_count
        color.b = blue_v/max_count
        self.pub.publish(color)

class SensorPublisher():
    def __init__(self, bus_id=1):
        rospy.init_node('sensor_pi_node', anonymous=True)
        self.i2c = smbus.SMBus(bus_id)
        self.node_list = list()

        """
        use_ultrasonic = rospy.get_param("~use_ultrasonic")
        use_tof = rospy.get_param("~use_tof")
        if use_ultrasonic:
            self.node_list.append(UltraSonicPublisher(self.i2c, 0x57))
        if use_tof:
            self.node_list.append(TOFPublisher(self.i2c, 0x29))
        color_sensor_pub0 = ColorSensorPublisher(self.i2c, TCS34725_ADDRESS, topic_name="color_value0")
        color_sensor_pub1 = ColorSensorPublisher(self.i2c, TCS34725_ADDRESS, topic_name="color_value1")
        self.node_list.append(I2CHubPublisher(self.i2c, TCA9548A_address, {0:color_sensor_pub0, 1:color_sensor_pub1}))
        """
        config_path  = rospy.get_param("~config_path")
        json_open = open('/catkin_ws/src/sensor_pi/config/all_sensor.json', 'r')
        config = json.load(json_open)
        for node_name, node_args in config.items():
            if node_name=='I2CHubPublisher':
                sensors = {}
                for i in range(0,8):
                    idx_name = str(i)
                    if idx_name in node_args:
                        cnode_args = node_args.pop(idx_name)
                        cnode_name = cnode_args.pop("name")
                        cnode_address = cnode_args.pop("address")
                        cnode_address = int(cnode_address, 16)
                        sensor_node = globals()[cnode_name](i2c=self.i2c, address=cnode_address, **cnode_args)
                        sensors[i] = sensor_node
                address = node_args.pop("address")
                if type(address) is str:
                    address = int(address, 16)
                node = globals()[node_name](i2c=self.i2c, address=address, sensor_dict=sensors, **node_args)
                self.node_list.append(node)

            else:
                address = node_args.pop("address")
                address = int(address, 16)
                node = globals()[node_name](i2c=self.i2c, address=address, **node_args)
                self.node_list.append(node)

        for node in self.node_list:
            node.setup()

    def loop(self):
        for node in self.node_list:
            node.loop()
        if len(self.node_list) == 0:
            time.sleep(0.1)


if __name__ == '__main__':
    sensor_pub = SensorPublisher()
    while not rospy.is_shutdown():
        sensor_pub.loop()
