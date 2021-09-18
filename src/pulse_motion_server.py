#! /usr/bin/env python
"""
pulse_motion.py
Zhiang Chen, Nov 2020
"""

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import actionlib
import pbr_gazebo.msg
from threading import Thread
from collections import deque
import numpy as np
from numpy import pi, sin

class PulseMotion(object):
    def __init__(self, action_name='pulse_motion_server'):
        self.default_vel = 0.0
        # joint state subscriber
        self.x = 0
        rospy.Subscriber("/prismatic_box_controller/joint_states", JointState, self.jointstate_cb)

        # velocity controller
        self.Hz = 200
        self.vel_pub = rospy.Publisher('/prismatic_box_controller/prismatic_joint_controller/command', Float64, queue_size=10)
        self.vel_command = self.default_vel
        self.vel_thread = Thread(target=self.send_vel, args=())
        self.vel_thread.daemon = True
        self.vel_thread.start()

        # pulse motion action server
        self._feedback = pbr_gazebo.msg.AFFeedback()
        self._result = pbr_gazebo.msg.AFResult()

        self._as = actionlib.SimpleActionServer(action_name, pbr_gazebo.msg.AFAction,
                                                execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
        rospy.loginfo("pulse_motion_planner/" + action_name + " has been initialized!")

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
            # pulse motion
            # displacement function: d = -A*cos(2*pi*F*t) + A
            # velocity function: v = 2*pi*A*F*sin(2*pi*F*t)
            # acceleration function: a = 4*pi^2*F^2*A*cos(2*pi*F*t)
            print(goal)
            T = 1. / F  # T is rock displacement period
            step_nm = int(T*self.Hz)+1  # self.Hz is control rate;
            # step_nm is publishing number for controller
            for j in range(step_nm):
                t = j*(1./self.Hz)
                self.vel_command = 2*pi*A*F*sin(2*pi*F*t)
                # print('t', t)
                # print('F', F)
                # print('A', A)
                # print(2*pi*t/F)
                # print(self.vel_command)
                # print('-')
                rate.sleep()

            self.vel_command = self.default_vel
            self._result.success = True
            self._as.set_succeeded(self._result)
            rospy.loginfo('pulse motion completed')

    def jointstate_cb(self, data):
        self.x = data.position[0]

    def send_vel(self):
        rate = rospy.Rate(self.Hz)  # Hz
        while not rospy.is_shutdown():
            self.vel_pub.publish(self.vel_command)
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node("pulse_motion_planner", anonymous=False)
    pulse_motion_planner = PulseMotion()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Node killed!")
