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
from gazebo_msgs.msg import LinkStates

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


        # joint state subscriber
        self.link_name = "prismatic_large_box::box"
        rospy.Subscriber('/gazebo/link_states', LinkStates, self.LinkStatecallback)
        self.shake_table_twist_pub = rospy.Publisher('/shake_table/twist', Twist, queue_size=10)
        self.shake_table_pose_pub = rospy.Publisher('/shake_table/pose', Pose, queue_size=10)

        self.pbr_pose = Pose()
        self.pbr_twist = Twist()
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback)

        rospy.loginfo('pluse_motion_smart_client has been initialized')

        plt.axis([0, 1.2, 0, 1.2])
        plt.xlabel('PGA (g)')
        plt.ylabel('PGV/PGA (s)')
        self.toppling_data = []

        rospy.loginfo('Start simulation!')

        FA_data = self.sampleMotionParam()
        print(FA_data)

        """        
        As = np.linspace(0.1, 0.5, 20)
        Fs = np.linspace(0.1, 1.5, 20)
        r = self.getRange(As, Fs)
        print(r)
        rospy.loginfo("Maximum PGA: " + str(r[0]))
        rospy.loginfo("Maximum PGV/PGA: " + str(r[2]))
        for A in As:
           for F in Fs:
        """

        for F, A in FA_data:
            self.runExperiment(A, F)

        nd_data = np.asarray(self.toppling_data)
        np.savetxt("toppling_data.csv", nd_data, delimiter=",")


    def LinkStatecallback(self, data):
        idx = data.name.index(self.link_name)
        double_rock_pose = data.pose[idx]
        double_rock_twist = data.twist[idx]
        self.shake_table_twist_pub.publish(double_rock_twist)
        self.shake_table_pose_pub.publish(double_rock_pose)


    def callback(self, data):
        model_name = 'rock_box_1_1_2'
        if model_name in data.name:
            idx = data.name.index('rock_box_1_1_2')
            self.pbr_pose = data.pose[idx]
            self.pbr_twist = data.twist[idx]

    def runExperiment(self, A, F):
        # load model
        initial_pose = Pose()
        initial_pose.position.x = 0
        initial_pose.position.y = 0
        initial_pose.position.z = 13.47
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
        rospy.sleep(4.)

        # delete model
        self.delete_client('rock_box_1_1_2')

        # reset shake table
        goal = pbr_gazebo.msg.AFGoal(A=0, F=0)
        self.client.send_goal(goal)
        self.client.wait_for_result()
        result = self.client.get_result()
        rospy.sleep(.5)
        
        # delete model
        #self.delete_client('rock_box_1_1_2')

        state = self.checkToppled()
        self.logData(A, F, state)

    def checkToppled(self):
        q = self.pbr_pose.orientation
        r, p, y = tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))
        #print(r,p,y)
        if (abs(r) + abs(p)) < 0.1:
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

    def sampleMotionParam(self):
        PGV_2_PGA = np.linspace(0.1, 1.0, 3)
        PGA = np.linspace(0.2, 1.1, 3)
        Fs = 1./(2*pi*PGV_2_PGA)
        FA_data = []
        for F in Fs:
            for pga in PGA:
                A = 9.807*pga/(4*pi**2*F**2)
                FA_data.append((F, A))

        FA_data = np.asarray(FA_data)
        return FA_data


if __name__ == '__main__':
    rospy.init_node('pluse_motion_smart_client', anonymous=False)
    smart_client = SmartClient()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Node killed!")
