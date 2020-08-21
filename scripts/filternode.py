#!/usr/bin/env python
from __future__ import print_function

import rospy
from ur5teleop.msg import daqdata
from math import cos, pi

def callback(data,args):
    alpha,pub=args
    angles_wrapped=angles(data)  # generate angles from reference voltage and encoder voltage
    angles_unwrapped=unwrap(angles_wrapped)  # generate unwrapped angles

def unwrap(angles_wrapped):
    try:
        prevangles_w, prevangles_uw= unwrap.prev  # if this fails it is the first iteration
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
    ref=data.ref
    maxv=ref-0.015
    minv=0
    voltages=[data.encoder1,data.encoder2,data.encoder3,data.encoder4,data.encoder5,data.encoder6]
    angles=[]

    for volt in voltages:
        ratio=(volt-minv)/(maxv-minv)
        if ratio > 1:
            ratio=1
        if ratio < 0:
            ratio=0
        angle=ratio*2*pi
        angles.append(angle)
    return angles

def listener():
    fs=100 # sample freq in hz
    fc=10 #corner freq in hz
    alpha=alphacalc(fs,fc)
    pub = rospy.Publisher('daqdata_filtered', daqdata, queue_size=10)
    rospy.init_node('filter')
    rospy.Subscriber("daqdata_raw", daqdata, callback,(alpha,pub))

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

def alphacalc(fs,fc):
    alpha=cos(2*pi*fc/fs)-1+(cos(2*pi*fc/fs)**2-4*cos(2*pi*fc/fs)+3)**0.5
    return alpha

if __name__ == '__main__':
    listener()
