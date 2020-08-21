#!/usr/bin/env python

from __future__ import print_function
from time import sleep
from os import system
from sys import stdout

from uldaq import (get_daq_device_inventory, DaqDevice, InterfaceType,
                   AiInputMode, AInFlag)

import rospy
from ur5teleop.msg import daqdata
from geometry_msgs.msg import Twist


def talker():
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist , queue_size=10 )
    rospy.init_node('daqnode')
    rate = rospy.Rate(60) # 10hz

    daq_device = None

    range_index = 0
    interface_type = InterfaceType.ANY
    low_channel = 0
    high_channel = 6

    try:
        # Get descriptors for all of the available DAQ devices.
        devices = get_daq_device_inventory(interface_type)
        number_of_devices = len(devices)
        if number_of_devices == 0:
            raise RuntimeError('Error: No DAQ devices found')

        if number_of_devices == 1:
            descriptor_index= 0

        else:
            print('Found', number_of_devices, 'DAQ device(s):')
            for i in range(number_of_devices):
                print('  [', i, '] ', devices[i].product_name, ' (',
                      devices[i].unique_id, ')', sep='')

            descriptor_index = input('\nPlease select a DAQ device, enter a number'
                                     + ' between 0 and '
                                     + str(number_of_devices - 1) + ': ')
            descriptor_index = int(descriptor_index)
            if descriptor_index not in range(number_of_devices):
                raise RuntimeError('Error: Invalid descriptor index')

        # Create the DAQ device from the descriptor at the specified index.
        daq_device = DaqDevice(devices[descriptor_index])

        # Get the AiDevice object and verify that it is valid.
        ai_device = daq_device.get_ai_device()
        if ai_device is None:
            raise RuntimeError('Error: The DAQ device does not support '
                               'analog input')

        # Establish a connection to the DAQ device.
        descriptor = daq_device.get_descriptor()
        #print('\nConnecting to', descriptor.dev_string, '- please wait...')
        # For Ethernet devices using a connection_code other than the default
        # value of zero, change the line below to enter the desired code.
        daq_device.connect(connection_code=0)

        ai_info = ai_device.get_info()

        # The default input mode is SINGLE_ENDED.
        input_mode = AiInputMode.SINGLE_ENDED
        # If SINGLE_ENDED input mode is not supported, set to DIFFERENTIAL.
        if ai_info.get_num_chans_by_mode(AiInputMode.SINGLE_ENDED) <= 0:
            input_mode = AiInputMode.DIFFERENTIAL

        # Get the number of channels and validate the high channel number.
        number_of_channels = ai_info.get_num_chans_by_mode(input_mode)
        if high_channel >= number_of_channels:
            high_channel = number_of_channels - 1

        # Get a list of supported ranges and validate the range index.
        ranges = ai_info.get_ranges(input_mode)
        if range_index >= len(ranges):
            range_index = len(ranges) - 1

        # print('\n', descriptor.dev_string, ' ready', sep='')
        # print('    Function demonstrated: ai_device.a_in()')
        # print('    Channels: ', low_channel, '-', high_channel)
        # print('    Input mode: ', input_mode.name)
        # print('    Range: ', ranges[range_index].name)
        # try:
        #     input('\nHit ENTER to continue\n')
        # except (NameError, SyntaxError):
        #     pass
        #
        # system('clear')
        print('connected to DAQ and starting publish node')
        while not rospy.is_shutdown():
            ref= ai_device.a_in(0, input_mode, ranges[range_index],AInFlag.DEFAULT)
            ch1= ai_device.a_in(1, input_mode, ranges[range_index],AInFlag.DEFAULT)   
            ch2= ai_device.a_in(2, input_mode, ranges[range_index],AInFlag.DEFAULT)   
            ch3= ai_device.a_in(3, input_mode, ranges[range_index],AInFlag.DEFAULT)   
            ch4= ai_device.a_in(4, input_mode, ranges[range_index],AInFlag.DEFAULT)   
            ch5= ai_device.a_in(5, input_mode, ranges[range_index],AInFlag.DEFAULT)   
            ch6= ai_device.a_in(6, input_mode, ranges[range_index],AInFlag.DEFAULT)         
            # rospy.loginfo([ref, ch1])
            vel_msg=Twist()
            vel_msg.linear.x=1
            vel_msg.linear.y=0
            vel_msg.linear.z=0

            vel_msg.angular.x=0
            vel_msg.angular.y=0
            # vel_msg.angular.z=ch1
            if ch1 > 2.5:
                vel_msg.angular.z=2

            else:
                vel_msg.angular.z=-2

            pub.publish(vel_msg)
            rate.sleep()

    finally:
        if daq_device:
            if daq_device.is_connected():
                daq_device.disconnect()
            daq_device.release()
        
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
