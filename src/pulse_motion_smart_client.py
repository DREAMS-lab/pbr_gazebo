#! /usr/bin/env python
"""
pulse_motion_smart_client.py
Zhiang Chen, Dec 2020
"""
from __future__ import print_function
import rospy
import actionlib
import os
import pbr_gazebo.msg
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import DeleteModel, SpawnModel
import rospkg
from geometry_msgs.msg import Point, Pose, Twist
import numpy as np
import tf
from numpy import pi
import matplotlib.pyplot as plt

class SmartClient(object):
    def __init__(self):
        """
        Example of loading and deleting a model:

        initial_pose = Pose()
        initial_pose.position.x = 0
        initial_pose.position.y = 0
        initial_pose.position.z = 4
        file_name = os.path.join(self.pkg_path, "models/rock_models/rock_box_1_1_2/model.sdf")
        model_name = 'rock_box_1_1_2'
        with open(file_name) as xml_file:
            sdf_f = xml_file.read()
        self.load_client(model_name, sdf_f, '', initial_pose, "world")
        rospy.sleep(10.)
        self.delete_client('rock_box_1_1_2')
        """
        rospack = rospkg.RosPack()
        rospack.list()
        self.pkg_path = rospack.get_path('pbr_gazebo')

        self.client = actionlib.SimpleActionClient('pulse_motion_server', pbr_gazebo.msg.AFAction)
        self.client.wait_for_server()
        rospy.loginfo('pulse motion server is connected')
        rospy.wait_for_service('/gazebo/delete_model')
        rospy.loginfo('gazebo delete_model server is connected')
        self.delete_client = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        rospy.loginfo('gazebo spawn_sdf_model server is connected')
        self.load_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

        self.pbr_pose = Pose()
        self.pbr_twist = Twist()
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback)

        rospy.loginfo('pluse_motion_smart_client has been initialized')

        plt.axis([0, 3, 0, 2])
        self.toppling_data = []

        rospy.loginfo('Start simulation!')
        As = np.linspace(0.1, 0.5, 20)
        Fs = np.linspace(0.1, 1.5, 20)
        r = self.getRange(As, Fs)
        print(r)
        rospy.loginfo("Maximum PGA: " + str(r[0]))
        rospy.loginfo("Maximum PGV/PGA: " + str(r[2]))
        for A in As:
            for F in Fs:
                # load model
                initial_pose = Pose()
                initial_pose.position.x = 0
                initial_pose.position.y = 0
                initial_pose.position.z = 4
                file_name = os.path.join(self.pkg_path, "models/rock_models/rock_box_1_1_2/model.sdf")
                model_name = 'rock_box_1_1_2'
                with open(file_name) as xml_file:
                    sdf_f = xml_file.read()
                self.load_client(model_name, sdf_f, '', initial_pose, "world")
                rospy.sleep(2.)

                # pulse motion
                goal = pbr_gazebo.msg.AFGoal(A=A, F=F)
                self.client.send_goal(goal)
                self.client.wait_for_result()
                result = self.client.get_result()
                rospy.sleep(2.)

                # delete model
                self.delete_client('rock_box_1_1_2')

                # reset shake table
                goal = pbr_gazebo.msg.AFGoal(A=0, F=0)
                self.client.send_goal(goal)
                self.client.wait_for_result()
                result = self.client.get_result()
                rospy.sleep(2.)

                state = self.checkToppled()
                self.logData(A, F, state)
        nd_data = np.asarray(self.toppling_data)
        np.savetxt("toppling_data.csv", nd_data, delimiter=",")


    def callback(self, data):
        model_name = 'rock_box_1_1_2'
        if model_name in data.name:
            idx = data.name.index('rock_box_1_1_2')
            self.pbr_pose = data.pose[idx]
            self.pbr_twist = data.twist[idx]

    def checkToppled(self):
        q = self.pbr_pose.orientation
        r, p, y = tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))
        if (abs(r) + abs(r)) < 0.1:
            return False
        else:
            return True

    def logData(self, A, F, state):
        PGV = 2*pi*A*F
        PGA = 4*pi**2*F**2*A
        PGV_2_PGA = PGV/PGA
        PGA_g = PGA/9.807
        if state:
            # toppled
            plt.scatter(PGA_g, PGV_2_PGA, c='r')
            self.toppling_data.append((PGA_g, PGV_2_PGA, 1))
        else:
            plt.scatter(PGA_g, PGV_2_PGA, c='b', marker="v")
            self.toppling_data.append((PGA_g, PGV_2_PGA, 0))
        plt.pause(0.05)


    def getRange(self, As, Fs):
        data = []
        for A in As:
            for F in Fs:
                PGV = 2 * pi * A * F
                PGA = 4 * pi ** 2 * F ** 2 * A
                PGV_2_PGA = PGV / PGA
                PGA_g = PGA / 9.807
                data.append((PGA_g, PGV_2_PGA))

        nd = np.asarray(data)
        return (nd[:, 0].max(), nd[:, 0].min(), nd[:, 1].max(), nd[:, 1].min())




if __name__ == '__main__':
    rospy.init_node('pluse_motion_smart_client', anonymous=False)
    smart_client = SmartClient()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Node killed!")