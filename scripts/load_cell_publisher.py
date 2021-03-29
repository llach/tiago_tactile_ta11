#!/usr/bin/env python

import u6
import rospy

import numpy as np

from _collections import deque
from tiago_tactile_msgs.msg import TA11
from tiago_tactile_msgs.srv import GetForceThreshold, GetForceThresholdResponse

topic = '/ta11'

rospy.init_node('ta11_tactile', anonymous=True)
pub = rospy.Publisher(topic, TA11, queue_size=1)

thresh_srv = None

numChannels = 2

SCALING = rospy.get_param("~scaling")
SMOOTHING = rospy.get_param("~smoothing")
gainIndex = rospy.get_param("~gainIndex")
calibrationTime = rospy.get_param("~calibrationTime")

resolutionIndex = 0
settlingFactor = 0

print(
    "\n~~~ TA11 Sensor Measurement ~~~\n\n" +
    "Parameters:\n" +
    "Scaling:\t {}\n".format(SCALING) +
    "Smoothing:\t {}\n".format(SMOOTHING) +
    "Gain:\t\t {}\n".format(gainIndex)
)

latest_values = [deque(maxlen=SMOOTHING) for _ in range(numChannels)]

d = u6.U6()
d.getCalibrationData()

sensor_frames = [
    "ta11_right_finger_link",
    "ta11_left_finger_link"
]

right_b = 0
left_b = 0

biased_right = []
biased_left = []

thresh = 0

def get_thresh(req):
    return GetForceThresholdResponse(threshold=thresh)

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

    feedbackArguments.append(u6.AIN24(0, resolutionIndex, gainIndex, settlingFactor, True))
    feedbackArguments.append(u6.AIN24(2, resolutionIndex, gainIndex, settlingFactor, True))

    cycle = 0

    calibrated = False

    rospy.loginfo("Calibrating sensors using {} samples ...".format(calibrationTime))
    while not rospy.is_shutdown():
        results = d.getFeedback(feedbackArguments)

        processed_values = []
        for i in range(numChannels):
            latest_values[i].append(d.binaryToCalibratedAnalogVoltage(gainIndex, results[2 + i]) * SCALING)

            _b = right_b if i == 0 else left_b
            processed_values.append(-1 * (np.mean(latest_values[i]) + _b))

        if not calibrated:
            cycle += 1

            biased_right.append(processed_values[0])
            biased_left.append(processed_values[1])

            if cycle > calibrationTime:
                calibrated = True

                right_b = np.mean(biased_right)
                left_b = np.mean(biased_left)

                thresh = np.max([np.abs(np.max(biased_right) - right_b), np.abs(np.max(biased_left) - left_b)]) * 1.05

                thresh_srv = rospy.Service('get_ta11_threshold', GetForceThreshold, get_thresh)

                rospy.loginfo("Calibration done! Bias Right: {} | Bias Left: {} | Threshold: {}".format(right_b, left_b, thresh))
                rospy.loginfo("Publishing sensor values on {}".format(topic))

                latest_values = [deque(maxlen=SMOOTHING) for _ in range(numChannels)]

        if calibrated:
            m = TA11()
            m.header.frame_id = "base_link"
            m.header.stamp = rospy.Time.now()

            m.frame_names = sensor_frames
            m.sensor_values = processed_values

            pub.publish(m)
finally:
    d.close()
