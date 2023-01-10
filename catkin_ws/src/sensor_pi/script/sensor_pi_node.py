#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import Float64, ColorRGBA, Float32MultiArray, MultiArrayDimension
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

from ads1015 import ADS1015

class ReflectorPublisher(I2CSensorPublisherNodeAbst):
    def __init__(self, i2c, address, pins, topic_name="reflector_value"):
        super(ReflectorPublisher, self).__init__(i2c, address)
        self.adc = ADS1015(i2c, address)
        self.pins = pins
        self.adc_ports = [self.adc.get_channel_data(pin) for pin in self.pins]
        self.pub = rospy.Publisher('~{}'.format(topic_name), Float32MultiArray, queue_size=1)

    def setup(self):
        None

    def loop(self):
        msg = Float32MultiArray()
        for adc_port in self.adc_ports:
            value = self.adc.read_se_adc(adc_port)
            msg.data.append(value)
        dim0 = MultiArrayDimension()
        dim0.label = ''
        dim0.size = len(self.adc_ports)
        dim0.stride = 1
        msg.layout.dim.append(dim0)
        self.pub.publish(msg)


class SensorPublisher():
    def __init__(self, bus_id=1):
        # self.i2c = smbus.SMBus(bus_id)
        bus_type = rospy.get_param("~bus_type")
        if bus_type ==  "smbus":
            bus_id = rospy.get_param("~bus_id")
            from smbusi2c import SMBusI2C
            self.i2c = SMBusI2C(bus_id)
        elif bus_type ==  "smbus2":
            bus_id = rospy.get_param("~bus_id")
            from smbus2i2c import SMBus2I2C
            self.i2c = SMBus2I2C(bus_id)
        elif bus_type ==  "ft232h":
            bus_id = rospy.get_param("~bus_id")
            from ft232hi2c import FT232HI2C
            self.i2c = FT232HI2C(bus_id)
        elif bus_type ==  "mcp2221":
            from mcp2221a import PyMCP2221A_I2C
            self.i2c = PyMCP2221A_I2C()
        else:
            assert False, "Not defined bus type"
        print("Bus setup end")
        self.node_list = list()

        config_path  = rospy.get_param("~config_path")
        print("config path : {}".format(config_path))
        json_open = open(config_path, 'r')
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
    rospy.init_node('sensor_pi_node', anonymous=True)
    sensor_pub = SensorPublisher()
    while not rospy.is_shutdown():
        sensor_pub.loop()
