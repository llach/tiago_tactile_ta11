import sys
import rospy
import actionlib

import numpy as np

from tiago_tactile_msgs.msg import TA11
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, JointTrajectoryControllerState


def generate_trajectory(first, last, num_points=5):
    total_time = 4.0

    jt = JointTrajectory()
    jt.header.frame_id = 'base_footprint'
    jt.joint_names = ['gripper_right_finger_joint', 'gripper_left_finger_joint']

    pts = []
    for t, j, i in zip(np.linspace(0, total_time, num_points), np.linspace(first[0], last, num_points),
                       np.linspace(first[1], last, num_points)):
        jp = JointTrajectoryPoint()
        jp.positions = [j, i]

        if t == 0.0: t += 0.1
        tm = rospy.Time(t)
        jp.time_from_start.secs = tm.secs
        jp.time_from_start.nsecs = tm.nsecs

        pts.append(jp)

    jt.points = pts
    return jt


sides = {
    0: "right",
    1: "left"
}

NOISE = 0.012

poses = [[], []]
forces = [[], []]

contact = False


def force_cb(fo):
    global contact
    global forces

    if not contact and np.all(np.abs(np.array(fo.sensor_values)) > NOISE):
        rospy.loginfo("got contact!")
        contact = True

    if contact:
        for i, v in enumerate(fo.sensor_values):
            forces[i].append(v)


def state_cb(state):
    global contact
    global poses

    if contact:
        for i, v in enumerate(state.actual.positions):
            poses[i].append(v)


rospy.init_node('estimate_k')
f_sub = rospy.Subscriber('ta11', TA11, callback=force_cb)
s_sub = rospy.Subscriber("/gripper_controller/state", JointTrajectoryControllerState, callback=state_cb)

ac = actionlib.SimpleActionClient('/gripper_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

rospy.loginfo("Waiting for JTC action server ...")
if not ac.wait_for_server(rospy.Duration(secs=5)):
    rospy.logerr("server not found!")
    sys.exit()


traj = generate_trajectory([0.044, 0.044], 0.0)
g = FollowJointTrajectoryGoal()
g.trajectory = traj

ac.send_goal_and_wait(g)

f_sub.unregister()
s_sub.unregister()

for k, v in sides.items():
    f_end = forces[k][-1]
    delta_p = poses[k][0] - poses[k][-1]
    k_ = f_end / delta_p

    print(v)
    print("got {} forces and {} positions".format(len(forces[k]), len(poses[k])))
    print("f_end = {}; delta_p = {};".format(f_end, delta_p))
    print("%%% k = {} %%%".format(k_))
    print("")