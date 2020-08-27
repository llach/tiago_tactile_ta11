#!/usr/bin/env python

import u6
import rospy

import numpy as np

from _collections import deque
from sensor_msgs.msg import Float64MultiArray

rospy.init_node('ta11_tactile', anonymous=True)
pub = rospy.Publisher('/ta11', TA11, queue_size=1)

numChannels = 2

SCALING = rospy.get_param("~scaling")
SMOOTHING = rospy.get_param("~smoothing")
resolutionIndex = rospy.get_param("~resolutionIndex")
gainIndex = rospy.get_param("~gainIndex")
settlingFactor = rospy.get_param("~settlingFactor")
differential = rospy.get_param("~differential")

latest_values = [deque(maxlen=SMOOTHING) for _ in range(numChannels)]

d = u6.U6()
d.getCalibrationData()

try:
    # Configure the IOs before the test starts
    rospy.loginfo("Preparing LabJack U6 ...")
    FIOEIOAnalog = (2 ** numChannels) - 1
    fios = FIOEIOAnalog & 0xFF
    eios = FIOEIOAnalog // 256

    d.getFeedback(u6.PortDirWrite(Direction=[0, 0, 0], WriteMask=[0, 0, 15]))

    feedbackArguments = []

    feedbackArguments.append(u6.DAC0_8(Value=125))
    feedbackArguments.append(u6.PortStateRead())

    feedbackArguments.append(u6.AIN24(0, resolutionIndex, gainIndex, settlingFactor, differential))
    feedbackArguments.append(u6.AIN24(2, resolutionIndex, gainIndex, settlingFactor, differential))

    rospy.loginfo("Publishing sensor values!")
    while not rospy.is_shutdown():
        results = d.getFeedback(feedbackArguments)

        for j in range(numChannels):
            latest_values[j].append(d.binaryToCalibratedAnalogVoltage(gainIndex, results[2 + j]) * SCALING)

        m = Float64MultiArray()
        m.data = [np.mean(dq) for dq in latest_values]

        pub.publish(m)
finally:
    d.close()
