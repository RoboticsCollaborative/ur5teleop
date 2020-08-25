#!/usr/bin/env python
from __future__ import print_function

import rospy
from ur5teleop.msg import daqdata, jointdata
from math import tan, pi


def callback(data, pub):
    angles_wrapped = angles(data)  # generate angles from reference voltage and encoder voltage
    angles_unwrapped = unwrap(angles_wrapped)  # generate unwrapped angles
    dt=getdt(data.sptime)
    angles_filtered = lowpass2(angles_unwrapped,dt)
    angular_vels=getvelocity(angles_filtered,dt)
    message=pubprep(angles_filtered,angular_vels,data.sptime,dt)
    pub.publish(message)

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


def lowpass2(vals,dt):
    fc_param = rospy.get_param('/frequency/corner')
    if not hasattr(callback,'fs'):
        lowpass2.fs=rospy.get_param('/frequency/sample')
        lowpass2.fc=fc_param
        lowpass2.coeffs=filtercoeffs(lowpass2.fs,lowpass2.fc)

    if fc_param != lowpass2.fc:
        lowpass2.fc = fc_param
        lowpass2.coeffs=filtercoeffs(lowpass2.fs,lowpass2.fc)

    b,a=lowpass2.coeffs

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


def listener():
    fc=rospy.get_param('/frequency/corner',default=5.0) # default corner frequency in Hz unless set by parameter prestartup
    rospy.set_param('/frequency/corner',fc) # sets the default filter corner frequency param
    pub = rospy.Publisher('daqdata_filtered',jointdata, queue_size=10)
    rospy.init_node('filter')
    rospy.Subscriber("daqdata_raw", daqdata, callback, pub)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
