#! /usr/bin/env python
"""
sin_motion.py
Zhiang Chen, Sept 2020
"""

import rospy
from geometry_msgs.msg import Twist
import numpy as np


if __name__ == '__main__':
    rospy.init_node('sin_motion', anonymous=False)
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    HZ = 30
    rate = rospy.Rate(HZ)  # controller publishing rate
    A = 1  # amplitude (meters)
    T = 0.3  # period (seconds). note this distinguishes from controller publishing rate
    t = 0  # initial horizon
    step = 1./HZ*(2*np.pi)/T
    while not rospy.is_shutdown():
        x = np.sin(t)
        vel = Twist()
        vel.linear.x = x
        pub.publish(vel)
        rate.sleep()
        t = t + step
