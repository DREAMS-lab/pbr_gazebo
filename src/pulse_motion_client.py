#! /usr/bin/env python
from __future__ import print_function
import rospy
import actionlib
import pbr_gazebo.msg
import sys


if __name__ == '__main__':
    A = float(sys.argv[1])
    F = float(sys.argv[2])
    rospy.init_node('pluse_motion_client', anonymous=False)
    client = actionlib.SimpleActionClient('pulse_motion_server', pbr_gazebo.msg.AFAction)
    client.wait_for_server()
    goal = pbr_gazebo.msg.AFGoal(A=A, F=F)
    client.send_goal(goal)
    client.wait_for_result()
    result = client.get_result()
    if result:
        print('action completed')
    else:
        print('action failed')