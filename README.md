# TA11 Sensor Readout 

This repository contains a [ROS node](https://github.com/llach/tiago_tactile_ta11/blob/master/launch/publish.launch) that reads sensor values from two TA11 sensors via a LabJack U6 and either publishes raw values or calibrated ones.

There are also some additional scripts to calculate some basic statistics of the values over *N* samples (mean, standard deviation and maximum) and some that make calibration easier.