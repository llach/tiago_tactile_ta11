#!/usr/bin/env python

import rospy
import u6
import time

import numpy as np
import seaborn as sns
import matplotlib.pyplot as plt

from _collections import deque
from tiago_tactile_msgs.msg import TA11


numSamples = 1000

sides = {
    0: "right",
    1: "left"
}

start_t = None
values = 2*[[]]

def plot_values():
    global start_t
    global values

    print("took {:.2f} seconds".format(time.time() - start_t))
    sns.distplot(values[0], hist=True, color='r', label='target values')
    plt.show()

    rospy.signal_shutdown("node done")

def cb(vals):
    global values
    global numSamples

    for i in range(2):
        values[i].append(vals.sensor_values[i])

    if len(values[0]) > numSamples:
        sub.unregister()
        plot_values()

rospy.init_node('value_dist')
sub = rospy.Subscriber('ta11', TA11, callback=cb)
start_t = time.time()

while not rospy.is_shutdown():
    rospy.spin()
