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
    A = 8  # amplitude (meters)
    T = 0.5  # period (seconds). note this distinguishes from controller publishing rate
    t = 0  # initial horizon
    step = 1./HZ*(2*np.pi)/T
    for _ in range(int(HZ*T)*4):
        x = A*np.sin(t)
        vel = Twist()
        vel.linear.x = x
        pub.publish(vel)
        rate.sleep()
        t = t + step

    for _ in range(10):
        vel = Twist()
        pub.publish(vel)
        rate.sleep()
