#!/usr/bin/env python

import sys
import time
import rospy

import numpy as np
import seaborn as sns
import matplotlib.pyplot as plt

from _collections import deque
from tiago_tactile_msgs.msg import TA11
sns.set(style="darkgrid")

numSamples = 10000

sides = {
    0: "right",
    1: "left"
}

start_t = None
values = [[], []]


def cb(vals):
    global values
    global numSamples

    for i, v in enumerate(vals.sensor_values):
        values[i].append(v)

    if len(values[0]) > numSamples:
        rospy.signal_shutdown("got enough samples")


rospy.init_node('value_dist')
sub = rospy.Subscriber('ta11', TA11, callback=cb)
start_t = time.time()

print("collecting {} samples ...".format(numSamples))

while not rospy.is_shutdown():
    rospy.spin()
sub.unregister()

print("took {:.2f} seconds\n".format(time.time() - start_t))
for k, v in sides.items():
    # sns.distplot(values[k], hist=True, color='b')
    # plt.title(v)
    # plt.show()

    max_val = np.max(np.abs(values[k]))
    mean = np.mean(values[k])
    std = np.std(values[k])

    print("{}\nmax = {:.5f}; mean = {:.5f}; std = {:.5f};\n".format(v, max_val, mean, std))
