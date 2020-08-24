#!/usr/bin/env python
from __future__ import print_function

import rospy
from ur5teleop.msg import daqdata
from math import cos, tan, pi
from geometry_msgs.msg import Vector3


def callback(data, args):
    pub, coeffs = args
    #sptime = rospy.Time.now() - data.sptime
    angles_wrapped = angles(data)  # generate angles from reference voltage and encoder voltage
    angles_unwrapped = unwrap(angles_wrapped)  # generate unwrapped angles
    angles_filtered = lowpass2(angles_unwrapped, coeffs)
    angular_vels=getvelocity(angles_filtered,data.sptime)
    print(angular_vels[0],angles_filtered[0])
    #print(sptime.secs)
    #pub.publish(angles_filtered[0], angles_unwrapped[0], 1225.0001)

def getvelocity(angles, sptime):
    try:
        prev_angles,prev_sptime=getvelocity.prev
    except:
        velocity=[0]*6
    else:
        dt=sptime-prev_sptime
        dtsecs=dt.secs+dt.nsecs*1e-9
        velocity=[]
        for i in range(6):
            dtheta=angles[i]-prev_angles[i]
            vel=dtheta/dtsecs
            velocity.append(vel)
    getvelocity.prev=(angles,sptime)
    return velocity






def lowpass2(vals, coeffs):
    b,a=coeffs
    try:
        prev_filter_vals,prev_vals=lowpass2.prev
    except:
        prev_filter_vals=[vals,vals]
        prev_vals=[vals,vals]
    filtered_vals=[]
    for i in range(6):
        filtered=b[0]*vals[i]+ b[1]*prev_vals[0][i] + b[2]*prev_vals[1][i]- a[0]*prev_filter_vals[0][i] - a[1]*prev_filter_vals[1][i]
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


def listener():
    fs = 100  # sample freq in hz
    fc = 5  # corner freq in hz
    coeffs = filtercoeffs(fs, fc)
    pub = rospy.Publisher('daqdata_filtered', Vector3, queue_size=10)
    rospy.init_node('filter')
    rospy.Subscriber("daqdata_raw", daqdata, callback, (pub, coeffs))

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


def filtercoeffs(fs, fc):
    zeta = 2 ** 0.5 / 2
    Ts = 1 / float(fs)
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
    listener()
