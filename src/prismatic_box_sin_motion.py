#! /usr/bin/env python
"""
sin_motion.py
Zhiang Chen, Oct 2020
"""

import rospy
from gazebo_msgs.srv import *

if __name__ == "__main__":
    rospy.init_node('sin_motion', anonymous=False)
    rospy.wait_for_service('/gazebo/apply_joint_effort')
    try:
        set_force_client = rospy.ServiceProxy('/gazebo/apply_joint_effort', ApplyJointEffort)
        force = ApplyJointEffortRequest()
        force.joint_name = 'box_joint'
        force.effort = 2000
        force.duration.set(2, 0)
        set_force_client(force)

    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)