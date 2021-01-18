#!/usr/bin/env python

import u6
import rospy

import numpy as np

from _collections import deque
from tiago_tactile_msgs.msg import TA11

topic = '/ta11_test'

rospy.init_node('ta11_tactile', anonymous=True)
pub = rospy.Publisher(topic, TA11, queue_size=1)

numChannels = 2

SCALING = 1000.0
SMOOTHING = 20
resolutionIndex = 1
gainIndex = 3
settlingFactor = 0
differential = True

right_m = 1.0
right_b = 0.08714

left_m = 1.0
left_b = 0.11807

print(
    "\n~~~ TA11 Sensor Measurement ~~~\n\n" +
    "Parameters:\n" +
    "Scaling:\t {}\n".format(SCALING) +
    "Smmothing:\t {}\n".format(SMOOTHING) +
    "ResIndex\t {}\n".format(resolutionIndex) +
    "Gain:\t\t {}\n".format(gainIndex) +
    "Settling:\t {}\n".format(settlingFactor) +
    "\n" +
    "right_m:\t {}\n".format(right_m) +
    "right_b:\t {}\n\n".format(right_b) +
    "left_m:\t\t {}\n".format(left_m) +
    "left_b:\t\t {}\n\n".format(left_b)
)


latest_values = [deque(maxlen=SMOOTHING) for _ in range(numChannels)]

d = u6.U6()
d.getCalibrationData()

sensor_frames = [
    "ta11_right_finger_link",
    "ta11_left_finger_link"
]

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

    rospy.loginfo("Publishing sensor values on {}".format(topic))
    while not rospy.is_shutdown():
        results = d.getFeedback(feedbackArguments)

        for j in range(numChannels):
            latest_values[j].append(d.binaryToCalibratedAnalogVoltage(gainIndex, results[2 + j]) * SCALING)

        m = TA11()
        m.header.frame_id = "base_link"
        m.header.stamp = rospy.Time.now()

        m.frame_names = sensor_frames
        for i in range(len(sensor_frames)):
            _m = right_m if i == 0 else left_m
            _b = right_b if i == 0 else left_b

            m.sensor_values.append(_m * np.mean(latest_values[i]) + _b)

        pub.publish(m)
finally:
    d.close()
