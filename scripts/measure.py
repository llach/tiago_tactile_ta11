#!/usr/bin/env python

import u6
import rospy
import signal
import argparse

import numpy as np

ITERATIONS = 10000

rospy.init_node('ta11_tactile', anonymous=True)

SCALING = rospy.get_param("~scaling")
SMOOTHING = rospy.get_param("~smoothing")
resolutionIndex = rospy.get_param("~resolutionIndex")
gainIndex = rospy.get_param("~gainIndex")
settlingFactor = rospy.get_param("~settlingFactor")
differential = rospy.get_param("~differential")

right_m = rospy.get_param("~right_m")
right_b = rospy.get_param("~right_b")

left_m = rospy.get_param("~left_m")
left_b = rospy.get_param("~left_b")

parser = argparse.ArgumentParser()
parser.add_argument('-c', '--calibrated', default=0, type=int)
parser.add_argument('-s', '--sensor', default=1, type=int)

args, _ = parser.parse_known_args()

# sensor 1 (default) is connected to AIN0, sensor 2 is connected to AIN2
sensor = 2 if args.sensor == 2 else 0
cal = False if args.calibrated == 0 else True
if sensor == 2:
    _m = left_m
    _b = left_b
else:
    _m = right_m
    _b = right_b

print(
    "\n~~~ TA11 Sensor Measurement ~~~\n\n" +
    "Parameters:\n" +
    "Scaling:\t {}\n".format(SCALING) +
    "Smmothing:\t {}\n".format(SMOOTHING) +
    "ResIndex\t {}\n".format(resolutionIndex) +
    "Gain:\t\t {}\n".format(gainIndex) +
    "Settling:\t {}\n".format(settlingFactor) +
    "\n" +
    "Sensor Config:\n"
    "Sensor:\t\t {}\n".format("LEFT (2)" if sensor == 2 else "RIGHT (1)") +
    "m:\t\t {}\n".format(_m) +
    "b:\t\t {}\n\n".format(_b) +
    "Values are {}CALIBRATED\n".format("NOT " if not cal else "")
)



d = u6.U6()
d.getCalibrationData()



def signal_handler(signal, frame):
  print("Quitting.")
  d.close()
  exit(0)
signal.signal(signal.SIGINT, signal_handler)

try:
    # Configure the IOs before the test starts
    rospy.loginfo("Preparing LabJack U6 ...")
    FIOEIOAnalog = 1
    fios = FIOEIOAnalog & 0xFF
    eios = FIOEIOAnalog // 256

    d.getFeedback(u6.PortDirWrite(Direction=[0, 0, 0], WriteMask=[0, 0, 15]))

    feedbackArguments = []

    feedbackArguments.append(u6.DAC0_8(Value=125))
    feedbackArguments.append(u6.PortStateRead())

    feedbackArguments.append(u6.AIN24(sensor, resolutionIndex, gainIndex, settlingFactor, differential))

    while True:
        latest_values = []
        try:
            raw_input("Press ENTER to start the measurement")
        except KeyboardInterrupt:
            print("Ok ok, quitting")
            exit(0)

        print("Collecting data ...")
        for _ in range(ITERATIONS):
            results = d.getFeedback(feedbackArguments)
            latest_values.append(d.binaryToCalibratedAnalogVoltage(gainIndex, results[2]) * SCALING)

        res = np.mean(latest_values)
        if cal:
            res = int(_m * (res + _b))
        print("Result ({}CALIBRATED)".format("NOT " if not cal else ""))
        print("{}{}".format(res, "g" if cal else ""))

finally:
    d.close()
