#!/usr/bin/env python
import rospy
import numpy as np
import time
# from __future__ import print_function
# from time import sleep
# from os import system
# from sys import stdout
from ur5teleop.msg import daqdata, jointdata

from uldaq import (get_daq_device_inventory, InterfaceType, ScanStatus,
                   ScanOption, CInScanFlag, CounterMeasurementType,
                   CounterMeasurementMode, CounterEdgeDetection,
                   CounterTickSize, CounterDebounceMode, CounterDebounceTime,
                   CConfigScanFlag, create_int_buffer, DaqDevice)

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

def display_scan_options(bit_mask):
    """Create a displays string for all scan options."""
    options = []
    if bit_mask == ScanOption.DEFAULTIO:
        options.append(ScanOption.DEFAULTIO.name)
    for option in ScanOption:
        if option & bit_mask:
            options.append(option.name)
    return ', '.join(options)


def get_supported_encoder_counters(ctr_info):
    """Create a list of supported encoder counters for the specified device."""
    encoders = []

    number_of_counters = ctr_info.get_num_ctrs()
    for counter_number in range(number_of_counters):
        measurement_types = ctr_info.get_measurement_types(counter_number)

        if CounterMeasurementType.ENCODER in measurement_types:
            encoders.append(counter_number)

    return encoders

def unwrap(angles_wrapped):
    try:
        prevangles_w, prevangles_uw = unwrap.prev  # if this fails it is the first iteration
    except:
        angles_unwrapped = angles_wrapped
    else:
        angles_unwrapped = []
        for i in range(6):
            dtheta = prevangles_w[i] - angles_wrapped[i]
            if dtheta > np.pi:  # adding to rotation
                change = 1
            elif dtheta < -np.pi:  # subtracting from the rotation
                change = -1
            else:
                change = 0
            rots = int(prevangles_uw[i] / (3.2 * np.pi)) + change
            if prevangles_uw[i] < 0:
                rots -= 1
            unwrapped = rots * 3.2 * np.pi + angles_wrapped[i]
            if unwrapped > prevangles_uw[i] + np.pi:
                rots -= 1
            if unwrapped < prevangles_uw[i] - np.pi:
                rots += 1
            angles_unwrapped.append(rots * 3.2 * np.pi + angles_wrapped[i])
    unwrap.prev = (angles_wrapped, angles_unwrapped)
    return angles_unwrapped

# def unwrap2(angles, prev_angles):
#     delta = angles - prev_angles
#
#     for i in range(len(delta)):
#         if delta > np.pi*2:
#             print('Negative detected...')


def counts_to_position(counts):
    return counts*(2*np.pi/20480)

class encoder_node():
    daq_device = None
    ctr_device = None
    status = ScanStatus.IDLE

    interface_type = InterfaceType.ANY
    low_encoder = 0
    encoder_count = 2
    sample_rate = 1000.0  # Hz
    samples_per_channel = 10000
    scan_options = ScanOption.CONTINUOUS
    scan_flags = CInScanFlag.DEFAULT

    encoder_type = CounterMeasurementType.ENCODER
    encoder_mode = (CounterMeasurementMode.ENCODER_X4)
                    # | CounterMeasurementMode.ENCODER_CLEAR_ON_Z)
    edge_detection = CounterEdgeDetection.RISING_EDGE
    tick_size = CounterTickSize.TICK_20ns
    debounce_mode = CounterDebounceMode.NONE
    debounce_time = CounterDebounceTime.DEBOUNCE_0ns
    config_flags = CConfigScanFlag.DEFAULT

    encoders = [0,1,2,3,4,5]
    # pub = rospy.Publisher('control_jointdata', jointdata, queue_size=1)

    rospy.init_node('digital_encoders')
    # time.sleep(5)
    rate = rospy.Rate(100)

    positions = None
    last_read_time = None

    pub = rospy.Publisher('digital_data', jointdata, queue_size=1)

    def __init__(self):
        rospy.on_shutdown(self.safe_shutdown)

        try:
            # Get descriptors for all of the available DAQ devices.
            devices = get_daq_device_inventory(self.interface_type)
            number_of_devices = len(devices)
            if number_of_devices == 0:
                raise RuntimeError('Error: No DAQ devices found')

            print('Found', number_of_devices, 'DAQ device(s):')
            # for i in range(number_of_devices):
            #     print('  [', i, '] ', devices[i].product_name, ' (',devices[i].unique_id, ')', sep='')
            descriptor_index = 0
            print('Selecting Daq # {}'.format(descriptor_index))

            # Create the DAQ device from the descriptor at the specified index.
            self.daq_device = DaqDevice(devices[descriptor_index])

            # Get the CtrDevice object and verify that it is valid.
            self.ctr_device = self.daq_device.get_ctr_device()
            if self.ctr_device is None:
                raise RuntimeError('Error: The DAQ device does not support '
                                   'counters')
            # Verify the specified device supports hardware pacing for counters.
            ctr_info = self.ctr_device.get_info()
            if not ctr_info.has_pacer():
                raise RuntimeError('\nError: The specified DAQ device does not '
                                   'support hardware paced counter input')

            # Establish a connection to the DAQ device.
            descriptor = self.daq_device.get_descriptor()
            print('\nConnecting to', descriptor.dev_string, '- please wait...')
            # For Ethernet devices using a connection_code other than the default
            # value of zero, change the line below to enter the desired code.
            self.daq_device.connect(connection_code=0)

            # Get the encoder counter channels.
            self.encoder_counters = get_supported_encoder_counters(ctr_info)
            if not self.encoder_counters:
                raise RuntimeError('\nError: The specified DAQ device does not '
                                   'support encoder channels')

            # self.low_encoder = 0
            # self.high_encoder = self.low_encoder + self.encoder_count - 1

            # Clear the counter, and configure the counter as an encoder.
            for encoder in self.encoders:
            # for encoder in range(self.low_encoder,self.high_encoder+1):
                self.ctr_device.c_config_scan(encoder, self.encoder_type, self.encoder_mode,
                                         self.edge_detection, self.tick_size, self.debounce_mode,
                                         self.debounce_time, self.config_flags)

            # Allocate a buffer to receive the data.
            self.data = create_int_buffer(len(self.encoders), self.samples_per_channel)

            # Start the scan
            # low_encoder = np.min()
            self.ctr_device.c_in_scan(self.encoders[0], self.encoders[-1], self.samples_per_channel,
                                 self.sample_rate, self.scan_options, self.scan_flags, self.data)


        except RuntimeError as error:
            print('\n', error)
            self.safe_shutdown()


    def safe_shutdown(self):
        if self.daq_device:
            if self.status == ScanStatus.RUNNING:
                self.ctr_device.scan_stop()
            if self.daq_device.is_connected():
                self.daq_device.disconnect()
            self.daq_device.release()

    def loop(self):

        self.status, transfer_status = self.ctr_device.get_scan_status()
        index = transfer_status.current_index
        self.last_read_time = rospy.Time.now()
        self.counts = np.array([self.data[index + encoder_index] for encoder_index in self.encoders])
        self.positions = np.array(unwrap(counts_to_position(self.counts)))
        while not rospy.is_shutdown():
            self.rate.sleep()
            self.status, transfer_status = self.ctr_device.get_scan_status()
            index = transfer_status.current_index
            read_time = rospy.Time.now()
            counts = np.array([self.data[index + encoder_index] for encoder_index in self.encoders])
            positions = np.array(unwrap(counts_to_position(counts)))

            dt = read_time - self.last_read_time
            dt_seconds = dt.secs + dt.nsecs * 1e-9


            velocities = (positions-self.positions)/dt_seconds
            # j_data = jointdata()
            self.pub.publish(pubprep(positions, velocities, read_time, dt_seconds))
            # print(counts[0],counts_to_position(counts)[0],positions[0])
            self.last_read_time = read_time
            self.positions = positions
            self.counts = counts

    def read_data(self):
        self.status, transfer_status = self.ctr_device.get_scan_status()
        read_time = rospy.Time.now()
        index = transfer_status.current_index
        positions = [self.data[index + encoder_index] for encoder_index in self.encoders]



        return positions, velocities
        # for encoder_index in range(self.encoder_count):
        #     print('chan =', (encoder_index), ': ',
        #           '{:.6f}'.format(self.data[index + encoder_index]))

if __name__ == "__main__":

    node = encoder_node()
    node.loop()
    # rospy.on_shutdown(node.safe_shutdown)
    # time.sleep(5)
    # rate = rospy.Rate(10)
    # while not rospy.is_shutdown():
    #     node.read_data()
