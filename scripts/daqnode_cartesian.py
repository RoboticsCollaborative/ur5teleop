#!/usr/bin/env python

import rospy
import rospkg
import numpy as np
import time
import uldaq

from daq_util import counts_to_position, PythonFilter, NumpyFilter, pubPose, pubTwist
from scipy.spatial.transform import Rotation as R
from ur5teleop.msg import daqdata, jointdata
from geometry_msgs.msg import Pose, Twist

from deadman_publisher import Deadman_Publisher

# Import the module
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from pykdl_utils.kdl_kinematics import KDLKinematics

r = rospkg.RosPack()
path = r.get_path('ur5teleop')
dummy_arm = URDF.from_xml_file(path+"/config/dummy_arm.urdf")

#48 bit counter
highest_count = 2**48
#detects and fixes rollover in the counter
def simple_counter_rolover(previous_value, input_value):
    diff = input_value - previous_value
    if np.abs(diff) > highest_count/2: # one rotation 20480
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
    debounce_mode = uldaq.CounterDebounceMode.TRIGGER_AFTER_STABLE
    debounce_time = uldaq.CounterDebounceTime.DEBOUNCE_7500ns
    config_flags = uldaq.CConfigScanFlag.DEFAULT

    encoders = [0,1,2,3,4,5]

    rospy.init_node('digital_encoders')
    sample_rate = 100
    rate = rospy.Rate(sample_rate)

    positions = None
    last_read_time = None

    pubp = rospy.Publisher('dummy_arm_pose', Pose, queue_size=1)
    pubv = rospy.Publisher('dummy_arm_twist', Twist, queue_size=1)

    # fc = [10.0, 10.0, 10.0, 10.0, 10.0, 10.0]
    # fs = sample_rate
    fs=rospy.get_param("/frequency/sample", default=100.0) # default sample freqency in hz unless defined presartup parameter
    rospy.set_param("/frequency/sample",fs)
    fc=rospy.get_param("/frequency/corner", default=[5.0, 5.0, 5.0, 10.0, 10.0, 10.0])# default corner frequency
    rospy.set_param("/frequency/corner",fc)
    filter = PythonFilter(fc,fs)
    # filter = NumpyFilter(fc,fs)

    tree = kdl_tree_from_urdf_model(dummy_arm)
    chai = tree.getChain("base_link", "wrist_3_link")
    kdl_kin = KDLKinematics(dummy_arm, "base_link", "wrist_3_link")

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

        #start deadman publisher
        dp = Deadman_Publisher(self.daq_device)

        while not rospy.is_shutdown():
            self.rate.sleep()
            read_time = rospy.Time.now()
            counts = np.array([simple_counter_rolover(self.counts[encoder], self.counter.c_in(encoder)) for encoder in self.encoders])
            # counts = np.array([self.counter.c_in(encoder) for encoder in self.encoders])
            positions = np.array(counts_to_position(counts))
            filtered_positions = np.array(self.filter.filter(positions))
            dt = read_time - self.last_read_time
            dt_seconds = dt.secs + dt.nsecs * 1e-9
            velocities = (filtered_positions-self.positions)/dt_seconds
            Ja = self.kdl_kin.jacobian(filtered_positions)
            FK = self.kdl_kin.forward(filtered_positions)
            Rt = FK[:3,:3]
            rot = R.from_dcm(Rt)
            vel_cartesian = np.array(np.matmul(Ja, velocities)).reshape(-1)
            pos_cartesian = np.array((FK[:3,3])).reshape(-1)
            ori_quaternion = rot.as_quat()
            # print(pos_cartesian[0])
            self.pubp.publish(pubPose(pos_cartesian,ori_quaternion))
            self.pubv.publish(pubTwist(vel_cartesian))
            self.last_read_time = read_time
            self.positions = filtered_positions
            self.counts = counts
            dp.sample_publish()

if __name__ == "__main__":

    node = encoder_node()
    rospy.loginfo("Started encoder node")
    node.loop()
