#!/usr/bin/env python
"""
gazebo pbr state extractor
Zhiang Chen
Nov 2020
"""

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
import csv


class StateExtractor(object):
    def __init__(self, timer_duration=0.5):
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback)
        self.command = None
        self.data_file = open('pbr_state_file.csv', mode='w')
        self.pbr_state_writer = csv.writer(self.data_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        self.pbr_state_writer.writerow(['step','pos_x', 'pos_y', 'pos_z', 'pos_X', 'pos_Y', 'pos_Z', 'pos_W', 'vel_x', 'vel_y', 'vel_z', 'vel_X', 'vel_Y', 'vel_Z'])
        self.step = 0
        self.pbr_pose = Pose()
        self.pbr_twist = Twist()
        self.timer = rospy.Timer(rospy.Duration(timer_duration), self.timerCallback)
        print("pbr_state_extractor initialized")

    def callback(self, data):
        idx = data.name.index('double_rock_pbr')
        self.pbr_pose = data.pose[idx]
        self.pbr_twist = data.twist[idx]
        #state_list = self.get_state_list()
        #self.pbr_state_writer.writerow(state_list)

    def timerCallback(self, timer):
        state_list = self.get_state_list()
        #print(state_list)
        self.pbr_state_writer.writerow(state_list)

    def get_state_list(self):
        pos_x = self.pbr_pose.position.x
        pos_y = self.pbr_pose.position.y
        pos_z = self.pbr_pose.position.z
        pos_X = self.pbr_pose.orientation.x
        pos_Y = self.pbr_pose.orientation.y
        pos_Z = self.pbr_pose.orientation.z
        pos_W = self.pbr_pose.orientation.w
        vel_x = self.pbr_twist.linear.x
        vel_y = self.pbr_twist.linear.y
        vel_z = self.pbr_twist.linear.z
        vel_X = self.pbr_twist.angular.x
        vel_Y = self.pbr_twist.angular.y
        vel_Z = self.pbr_twist.angular.z
        state_list = [self.step, pos_x, pos_y, pos_z, pos_X, pos_Y, pos_Z, pos_W, vel_x, vel_y, vel_z, vel_X, vel_Y, vel_Z]
        self.step += 1
        return state_list



if __name__ == '__main__':
    rospy.init_node('pbr_state_extractor', anonymous=False)
    state_extractor = StateExtractor(timer_duration=0.2)  # it shouldn't be faster than the publisher rate
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Node killed!")


