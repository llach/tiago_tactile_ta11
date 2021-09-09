#!/usr/bin/env python

import numpy as np
import rospy
import tf

from std_msgs.msg import Float64

# rotate vector v1 by quaternion q1
def qv_mult(q1, v1):
    v1 = tf.transformations.unit_vector(v1)
    q2 = list(v1)
    q2.append(0.0)
    return tf.transformations.quaternion_multiply(
        tf.transformations.quaternion_multiply(q1, q2),
        tf.transformations.quaternion_conjugate(q1)
    )[:3]

if __name__ == '__main__':
    rospy.init_node('gripper_gravity_comp')
    pub = rospy.Publisher('gripper_gravity_comp', Float64, queue_size=1)

    listener = tf.TransformListener()
    
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform('base_link', 'gripper_grasping_frame', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
    
        gravaxis = np.array([0, 0, -1]) #-z-axis
    
        n1_base = qv_mult(rot, [0,1,0])
        dof_projection = np.dot([0,0,1], n1_base)
    
        pub.publish(Float64(dof_projection))
    
        rate.sleep()