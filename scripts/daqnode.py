#!/usr/bin/env python

from __future__ import print_function
from time import sleep
from os import system
from sys import stdout

from uldaq import (get_daq_device_inventory, DaqDevice, InterfaceType,
                   AiInputMode, AInFlag)

import rospy
from ur5teleop.msg import daqdata, jointdata
from math import tan, pi

def talker():
    pub = rospy.Publisher('daqdata_filtered', jointdata, queue_size=1)
    rospy.init_node('daqnode')
    fs=rospy.get_param("/frequency/sample", default=100.0) # default sample freqency in hz unless defined presartup parameter
    rospy.set_param("/frequency/sample",fs)
    fc=rospy.get_param("/frequency/corner", default=[2.0, 2.0, 2.0, 5.0, 5.0, 5.0])# default corner frequency
    rospy.set_param("/frequency/corner",fc)




    rate = rospy.Rate(fs)

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
            class data:
                sptime= rospy.Time.now()
                ref= ai_device.a_in(0, input_mode, ranges[range_index],AInFlag.DEFAULT)
                encoder1= ai_device.a_in(1, input_mode, ranges[range_index],AInFlag.DEFAULT)
                encoder2= ai_device.a_in(2, input_mode, ranges[range_index],AInFlag.DEFAULT)
                encoder3= ai_device.a_in(3, input_mode, ranges[range_index],AInFlag.DEFAULT)
                encoder4= ai_device.a_in(4, input_mode, ranges[range_index],AInFlag.DEFAULT)
                encoder5= ai_device.a_in(5, input_mode, ranges[range_index],AInFlag.DEFAULT)
                encoder6= ai_device.a_in(6, input_mode, ranges[range_index],AInFlag.DEFAULT)

            angles_wrapped = angles(data)
            angles_unwrapped = unwrap(angles_wrapped)  # generate unwrapped angles
            dt = getdt(data.sptime)
            angles_filtered = lowpass2(angles_unwrapped, fs, fc)
            angular_vels = getvelocity(angles_filtered, dt)
            message = pubprep(angles_filtered, angular_vels, data.sptime, dt)
            pub.publish(message)
            rate.sleep()

    finally:
        if daq_device:
            if daq_device.is_connected():
                daq_device.disconnect()
            daq_device.release()

def getdt(sptime):
    try:
        prev_sptime=getdt.prev
    except:
        dtsecs=0
    else:
        dt=sptime-prev_sptime
        dtsecs = dt.secs + dt.nsecs * 1e-9
    getdt.prev=sptime
    return dtsecs


def pubprep(angles,vels,sptime,dtsecs):
    msg=jointdata()

    msg.sptime=sptime
    msg.dt=dtsecs
    msg.encoder1.pos=angles[0]
    msg.encoder1.vel=vels[0]

    msg.encoder2.pos=angles[1]
    msg.encoder2.vel=vels[1]

    msg.encoder3.pos=angles[2]
    msg.encoder3.vel=vels[2]

    msg.encoder4.pos=angles[3]
    msg.encoder4.vel=vels[3]

    msg.encoder5.pos=angles[4]
    msg.encoder5.vel=vels[4]

    msg.encoder6.pos=angles[5]
    msg.encoder6.vel=vels[5]

    return msg


def getvelocity(angles, dt):
    try:
        prev_angles=getvelocity.prev
    except:
        velocity=[0]*6
    else:
        velocity=[]
        for i in range(6):
            dtheta=angles[i]-prev_angles[i]
            vel=dtheta/dt
            velocity.append(vel)
    getvelocity.prev=angles
    return velocity


def lowpass2(vals,fs,fc):
    if not hasattr(lowpass2,'coeffs'):
        lowpass2.coeffs=[filtercoeffs(fs,fc[i]) for i in range(6)]

    # b,a=lowpass2.coeffs
    b = [lowpass2.coeffs[i][0] for i in range(6)]
    a = [lowpass2.coeffs[i][1] for i in range(6)]

    try:
        prev_filter_vals,prev_vals=lowpass2.prev
    except:
        prev_filter_vals=[vals,vals]
        prev_vals=[vals,vals]
    filtered_vals=[]
    for i in range(6):
        filtered=b[i][0]*vals[i]+ b[i][1]*prev_vals[0][i] + b[i][2]*prev_vals[1][i]- a[i][0]*prev_filter_vals[0][i] - a[i][1]*prev_filter_vals[1][i]
        filtered_vals.append(filtered)
    prev_filter_vals=[filtered_vals,prev_filter_vals[0]]
    prev_vals=[vals,prev_vals[0]]
    lowpass2.prev=(prev_filter_vals,prev_vals)
    return filtered_vals


def unwrap(angles_wrapped):
    try:
        prevangles_w, prevangles_uw = unwrap.prev  # if this fails it is the first iteration
    except:
        angles_unwrapped = angles_wrapped
    else:
        angles_unwrapped = []
        for i in range(6):
            dtheta = prevangles_w[i] - angles_wrapped[i]
            if dtheta > pi:  # adding to rotation
                change = 1
            elif dtheta < -pi:  # subtracting from the rotation
                change = -1
            else:
                change = 0
            rots = int(prevangles_uw[i] / (2 * pi)) + change
            if prevangles_uw[i] < 0:
                rots -= 1
            unwrapped = rots * 2 * pi + angles_wrapped[i]
            if unwrapped > prevangles_uw[i] + pi:
                rots -= 1
            if unwrapped < prevangles_uw[i] - pi:
                rots += 1
            angles_unwrapped.append(rots * 2 * pi + angles_wrapped[i])
    unwrap.prev = (angles_wrapped, angles_unwrapped)
    return angles_unwrapped


def angles(data):
    ref = data.ref
    maxv = ref - 0.015
    minv = 0
    voltages = [data.encoder1, data.encoder2, data.encoder3, data.encoder4, data.encoder5, data.encoder6]
    angles = []

    for volt in voltages:
        ratio = (volt - minv) / (maxv - minv)
        if ratio > 1:
            ratio = 1
        if ratio < 0:
            ratio = 0
        angle = ratio * 2 * pi
        angles.append(angle)
    return angles


def filtercoeffs(fs, fc):
    Ts=1/float(fs)
    zeta = 2 ** 0.5 / 2
    w0 = 2 * pi * fc
    K = w0 / tan(w0 * Ts / 2)
    phi = K ** 2 + 2 * zeta * w0 * K + w0 ** 2
    b0 = w0 ** 2 / phi
    b1 = 2 * b0
    b2 = b0
    a1 = (2 * w0 ** 2 - 2 * K ** 2) / phi
    a2 = (K ** 2 - 2 * zeta * w0 * K + w0 ** 2) / phi

    coeffs = ([b0, b1, b2], [a1, a2])
    return coeffs

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
