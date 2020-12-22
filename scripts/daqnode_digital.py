#!/usr/bin/env python

import rospy
import numpy as np
import time
import uldaq

from daq_util import pubprep, counts_to_position, PythonFilter, NumpyFilter
from ur5teleop.msg import daqdata, jointdata

#48 bit counter
highest_count = 2**48
#detects and fixes rollover in the counter
def simple_counter_rolover(previous_value, input_value):
    diff = input_value - previous_value
    if np.abs(diff) > 20480: #one revolution
        if diff > 0:
            output_value = input_value - highest_count
        else:
            output_value = highest_count + input_value
    else:
        output_value = input_value
    return output_value

# def unwrap_counter(previous_values, input_values):
#     return np.array([simple_counter_rolover(previous_values[i],input_values[i]) for i in range(len(input_values))])

def resolve_initial_count(value):
    if value > highest_count/2:
        return highest_count - value
    return value

class encoder_node():
    daq_device = None

    encoder_type = uldaq.CounterMeasurementType.ENCODER
    encoder_mode = (uldaq.CounterMeasurementMode.ENCODER_X4)
    edge_detection = uldaq.CounterEdgeDetection.RISING_EDGE
    tick_size = uldaq.CounterTickSize.TICK_20ns
    debounce_mode = uldaq.CounterDebounceMode.NONE
    debounce_time = uldaq.CounterDebounceTime.DEBOUNCE_0ns
    config_flags = uldaq.CConfigScanFlag.DEFAULT

    encoders = [0,1,2,3,4,5]

    rospy.init_node('digital_encoders')
    sample_rate = 100
    rate = rospy.Rate(sample_rate)

    positions = None
    last_read_time = None

    pub = rospy.Publisher('digital_data', jointdata, queue_size=1)

    fc = [10.0, 10.0, 10.0, 10.0, 10.0, 10.0]
    fs = sample_rate
    filter = NumpyFilter(fc,fs)

    def __init__(self):
        rospy.on_shutdown(self.safe_shutdown)

        try:
            # 1 = USB interface
            descriptors = uldaq.get_daq_device_inventory(uldaq.InterfaceType.USB, number_of_devices=1)

            self.daq_device = uldaq.DaqDevice(descriptors[0])
            # self.daq_device.connect()
            self.daq_device.connect(connection_code=0)

            self.counter = self.daq_device.get_ctr_device()
            # Clear the counter, and configure the counter as an encoder.
            for encoder in self.encoders:
            # for encoder in range(self.low_encoder,self.high_encoder+1):
                self.counter.c_config_scan(encoder, self.encoder_type, self.encoder_mode,
                                         self.edge_detection, self.tick_size, self.debounce_mode,
                                         self.debounce_time, self.config_flags)

        except RuntimeError as error:
            print('\n', error)
            self.safe_shutdown()


    def safe_shutdown(self):
        if self.daq_device:
            if self.daq_device.is_connected():
                self.daq_device.disconnect()
            self.daq_device.release()

    def loop(self):

        self.last_read_time = rospy.Time.now()
        self.counts = np.array([resolve_initial_count(self.counter.c_in(encoder)) for encoder in self.encoders])
        #TODO initialize filter
        self.positions = np.array(counts_to_position(self.counts))
        self.filter.calculate_initial_values(self.positions)
        while not rospy.is_shutdown():
            self.rate.sleep()
            read_time = rospy.Time.now()
            counts = np.array([simple_counter_rolover(self.counts[encoder], self.counter.c_in(encoder)) for encoder in self.encoders])
            positions = np.array(counts_to_position(counts))
            filtered_positions = np.array(self.filter.filter(positions))
            dt = read_time - self.last_read_time
            dt_seconds = dt.secs + dt.nsecs * 1e-9
            velocities = (filtered_positions-self.positions)/dt_seconds
            # print(filtered_positions)
            self.pub.publish(pubprep(filtered_positions, velocities, read_time, dt_seconds))
            self.last_read_time = read_time
            self.positions = filtered_positions
            self.counts = counts

if __name__ == "__main__":

    node = encoder_node()
    node.loop()
