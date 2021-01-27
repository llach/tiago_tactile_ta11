#!/usr/bin/env python

import u6
import time
import yaml

import numpy as np
import seaborn as sns
import matplotlib.pyplot as plt

from _collections import deque

conf = yaml.load(open('../config/ta11-mounted.yaml', 'r'))

numChannels = 2
numSamples = 100

SCALING = conf["scaling"]
SMOOTHING = conf["smoothing"]
resolutionIndex = conf["resolutionIndex"]
gainIndex = conf["gainIndex"]
settlingFactor = conf["settlingFactor"]
differential = conf["differential"]

right_m = conf["right_m"]
right_b = conf["right_b"]

left_m = conf["left_m"]
left_b = conf["left_b"]

print(
    "\n~~~ TA11 Sensor Measurement ~~~\n\n" +
    "Parameters:\n" +
    "Scaling:\t {}\n".format(SCALING) +
    "Smoothing:\t {}\n".format(SMOOTHING) +
    "ResIndex\t {}\n".format(resolutionIndex) +
    "Gain:\t\t {}\n".format(gainIndex) +
    "Settling:\t {}\n".format(settlingFactor) +
    "\n" +
    "right_m:\t {}\n".format(right_m) +
    "right_b:\t {}\n\n".format(right_b) +
    "left_m:\t\t {}\n".format(left_m) +
    "left_b:\t\t {}\n\n".format(left_b)
)

sides = {
    0: "right",
    1: "left"
}


values = 2*[[]]
latest_values = [deque(maxlen=SMOOTHING) for _ in range(numChannels)]

d = u6.U6()
d.getCalibrationData()

try:
    # Configure the IOs before the test starts
    print("Preparing LabJack U6 ...")
    FIOEIOAnalog = (2 ** numChannels) - 1
    fios = FIOEIOAnalog & 0xFF
    eios = FIOEIOAnalog // 256

    d.getFeedback(u6.PortDirWrite(Direction=[0, 0, 0], WriteMask=[0, 0, 15]))

    feedbackArguments = []

    feedbackArguments.append(u6.DAC0_8(Value=125))
    feedbackArguments.append(u6.PortStateRead())

    feedbackArguments.append(u6.AIN24(0, resolutionIndex, gainIndex, settlingFactor, differential))
    feedbackArguments.append(u6.AIN24(2, resolutionIndex, gainIndex, settlingFactor, differential))

    print("Recording {} samples ...".format(numSamples))
    start_t = time.time()

    while len(values[0]) < numSamples:
        results = d.getFeedback(feedbackArguments)

        for j in range(numChannels):
            latest_values[j].append(d.binaryToCalibratedAnalogVoltage(gainIndex, results[2 + j]) * SCALING)


        for i in range(2):
            _m = right_m if i == 0 else left_m
            _b = right_b if i == 0 else left_b
            print(_b)

            values[i].append(_m * np.mean(latest_values[i]) + _b)

    print("took {:.2f} seconds".format(time.time() - start_t))
    sns.distplot(values[0], hist=True, color='r', label='target values')
    plt.show()

finally:
    d.close()
