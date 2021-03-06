#!/usr/bin/env python

import os
import u6
import time
import rospy
import pickle

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
gainIndex = rospy.get_param("~gainIndex")

resolutionIndex = 0
settlingFactor = 0

calib_sensor = []
calib_contact = []

sensor_values = deque(maxlen=SMOOTHING)

d = u6.U6()
d.getCalibrationData()

_b = 0

max_con = 0
mu_con = 0

dev = 5

thresh = np.inf

resp_times = []

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

    loop_diff = 0

    rospy.loginfo("Starting ...")
    while not rospy.is_shutdown():
        startTime = datetime.now()
        results = d.getFeedback(feedbackArguments)

        contact = d.binaryToCalibratedAnalogVoltage(0, results[2])

        sensor_values.append(d.binaryToCalibratedAnalogVoltage(gainIndex, results[3]) * SCALING)
        force = (-1 * ((np.mean(sensor_values) + _b)))
        rt = (datetime.now() - startTime)

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
                in_contact = True
                did_trigger = False
                # startTime = datetime.now()
                loop_diff += 1

            # we're in contact and measured the force
            if in_contact and np.abs(force) > thresh and not did_trigger:
                # assumption: delay < 1sec
                resp_times.append(rt.microseconds)
                print(np.mean(resp_times), len(resp_times))
                did_trigger = True

            # contact lost
            if not has_contact and in_contact:
                in_contact = False
                loop_diff = 0

            cpub.publish(has_contact)
            spub.publish(force)
finally:
    print('bye')
    with open('{}/resp_times.pkl'.format(os.environ['HOME']), 'w') as f:
        pickle.dump(resp_times, f)
    d.close()
