#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from uldaq import (get_daq_device_inventory, DaqDevice, InterfaceType,
                   DigitalDirection, AiInputMode, AInFlag)

'''Enable sqitch class usage. Can be run as standalone, by running this script
with rosrun, or incorporated into the daqnode script, by calling sample_publish
inside the main loop'''

class Deadman_Publisher():
    enabled = False
    def __init__(self, daq_device, rate = 100):
        '''expects a DaqDevice object'''
        self.rate = rate

        # Get the DioDevice object and verify that it is valid.
        self.dio_device = daq_device.get_dio_device()
        if self.dio_device is None:
            raise RuntimeError('Error: The DAQ device does not support digital '
                               'input')
        #configure gpio pins
        dio_info = self.dio_device.get_info()
        port_types = dio_info.get_port_types()

        self.port_to_read = port_types[0] #reading bits from port A (pins 21-28)

        # Get the port I/O type and the number of bits for the first port.
        port_info = dio_info.get_port_info(self.port_to_read)

        # configure the entire port for input.
        self.dio_device.d_config_port(self.port_to_read, DigitalDirection.INPUT)

        #setup publisher
        self.deadman_pub = rospy.Publisher('enable_move', Bool, queue_size=1)
        #rospy.init_node('daqnode')

    def sample_publish(self):
        input=self.dio_device.d_bit_in(self.port_to_read,0)
        self.deadman_pub.publish(Bool(data = not input))

    def run(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.sample_publish()
            rate.sleep()

if __name__ == '__main__':
    devices = get_daq_device_inventory(InterfaceType.ANY)
    daq_device = DaqDevice(devices[0])
    daq_device.connect(connection_code=0)
    dp = Deadman_Publisher(daq_device, rate = 100)
    dp.run()
