#! /usr/bin/env python
from __future__ import print_function
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import actionlib
import pbr_gazebo.msg
from threading import Thread
from collections import deque
import numpy as np
import rospkg
import os
import csv
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist

class MotionFromFile(object):
    def __init__(self, file_name, link_name='double_rock::box'):
        self.link_name = link_name
        rospack = rospkg.RosPack()
        rospack.list()
        pkg_path = rospack.get_path('pbr_gazebo')
        self.file_name = os.path.join(pkg_path, 'src/ground_motion_data', file_name)
        times = []
        self.vel_commands = []
        with open(self.file_name, mode='r') as csv_file:
            csv_reader = csv.DictReader(csv_file)
            for row in csv_reader:
                t = float(row['time'])
                a = float(row['filtered_acc'])
                v = float(row['filtered_velocity'])
                d = float(row['displacement'])
                times.append(t)
                self.vel_commands.append(v)

        self.times = []
        for i in range(len(times) - 1):
            self.times.append(times[i+1] - times[i])

        # joint state subscriber
        rospy.Subscriber('/gazebo/link_states', LinkStates, self.LinkStatecallback)
        self.default_vel = 0.0
        self.x = 0
        rospy.Subscriber("/prismatic_box_controller/joint_states", JointState, self.jointstate_cb)
        self.double_rock_twist_pub = rospy.Publisher('/motion_from_file/double_rock/twist', Twist, queue_size=10)
        self.double_rock_pose_pub = rospy.Publisher('/motion_from_file/double_rock/pose', Pose, queue_size=10)



        # velocity controller
        self.Hz = 1000
        self.vel_pub = rospy.Publisher('/prismatic_box_controller/prismatic_joint_controller/command', Float64,
                                       queue_size=10)
        self.vel_command = self.default_vel
        self.vel_thread = Thread(target=self.send_vel, args=())
        self.vel_thread.daemon = True
        self.vel_thread.start()

        # pulse motion action server
        self._feedback = pbr_gazebo.msg.AFFeedback()
        self._result = pbr_gazebo.msg.AFResult()

        self._as = actionlib.SimpleActionServer('ground_motion_server', pbr_gazebo.msg.AFAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
        rospy.loginfo("pulse_motion_planner/ground_motion_server" + " has been initialized!")

    def execute_cb(self, goal):
        A = goal.A
        F = goal.F

        rate = rospy.Rate(self.Hz)  # Hz
        if A*F == 0:
            # reset
            err = - self.x
            errs = deque(maxlen=5)
            errs.append(0)
            P = 1
            I = 0.2
            while abs(err)>0.001:
                self.vel_command = P*err + I*np.array(errs).mean()
                rate.sleep()
                err = - self.x
                errs.append(err)
            self.vel_command = self.default_vel
            self._result.success = True
            self._as.set_succeeded(self._result)
            rospy.loginfo('reset completed')
        else:
            step_nm = len(self.times)
            for j in range(step_nm):
                self.vel_command = self.vel_commands[j]
                rospy.sleep(self.times[j])

            self.vel_command = self.default_vel
            self._result.success = True
            self._as.set_succeeded(self._result)
            rospy.loginfo('ground motion completed')


    def jointstate_cb(self, data):
        # this is from the ros_controller. It's not ground truth
        self.x = data.position[0]

    def LinkStatecallback(self, data):
        idx = data.name.index(self.link_name)
        double_rock_pose = data.pose[idx]
        double_rock_twist = data.twist[idx]
        self.double_rock_twist_pub.publish(double_rock_twist)
        self.double_rock_pose_pub.publish(double_rock_pose)



    def send_vel(self):
        rate = rospy.Rate(self.Hz)  # Hz
        while not rospy.is_shutdown():
            self.vel_pub.publish(self.vel_command)
            rate.sleep()



if __name__ == '__main__':
    rospy.init_node('motion_from_file', anonymous=False)
    double_rock = 'double_rock::box'
    shake_table = 'prismatic_large_box::box'
    mff = MotionFromFile('RSN316.csv', shake_table)
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Node killed!")
