#!/usr/bin/env python

import u6
import time
import rospy

import numpy as np

from datetime import datetime
from _collections import deque
from std_msgs.msg import Bool, Float64


rospy.init_node('measure_delay_ta11', anonymous=True)

cpub = rospy.Publisher('contact', Bool, queue_size=1)
spub = rospy.Publisher('single_ta11', Float64, queue_size=1)

numChannels = 2

SCALING = rospy.get_param("~scaling")
SMOOTHING = rospy.get_param("~smoothing")
resolutionIndex = rospy.get_param("~resolutionIndex")
gainIndex = rospy.get_param("~gainIndex")
settlingFactor = rospy.get_param("~settlingFactor")

calib_sensor = []
calib_contact = []

sensor_values = deque(maxlen=SMOOTHING)

d = u6.U6()
d.getCalibrationData()

_b = 0

max_con = 0
mu_con = 0

dev = 50

thresh = np.inf

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

    # channel, resolutionIndex, gainIndex, settingFactor, differentialReadout?
    feedbackArguments.append(u6.AIN24(2, 0, 0, 0, False))
    feedbackArguments.append(u6.AIN24(0, resolutionIndex, gainIndex, settlingFactor, True))

    loop = 0

    in_contact = False
    did_trigger = False
    startTime = time.time()

    rospy.loginfo("Starting ...")
    while not rospy.is_shutdown():

        results = d.getFeedback(feedbackArguments)

        contact = d.binaryToCalibratedAnalogVoltage(0, results[2])

        sensor_values.append(d.binaryToCalibratedAnalogVoltage(gainIndex, results[3]) * SCALING)
        force = (-1 * (np.mean(sensor_values) + _b))

        if loop < 500:
            if loop == 0:
                print("Calibrating ...")

            calib_sensor.append(force)
            calib_contact.append(contact)

            loop += 1
        else:
            if loop == 500:
                _b = np.mean(calib_sensor)
                thresh = np.max(calib_sensor-_b) * (1 + (dev/100))

                max_con = np.max(calib_contact)
                mu_con = np.mean(calib_contact)

                print("Done. Force Bias {} | Force Thresh {} | contact mean {} | contact max {} ".format(_b, thresh, mu_con, max_con))
                loop += 1

            has_contact = contact > (max_con*1.05)

            # contact acquired
            if not in_contact and has_contact:
                print("circuit closed.")
                in_contact = True
                did_trigger = False
                startTime = datetime.now()

            # we're in contact and measured the force
            if in_contact and np.abs(force) > thresh and not did_trigger:
                print("_b {} | force {} | thresh {}".format(_b, force, thresh))
                print("delay: {}".format(datetime.now() - startTime))
                did_trigger = True

            # contact lost
            if not has_contact and in_contact:
                in_contact = False


            cpub.publish(has_contact)
            spub.publish(force)
finally:
    d.close()
