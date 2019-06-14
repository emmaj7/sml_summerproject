#!/usr/bin/env python

"""
ROS interface object for localization with Qualisys odom.

This module simply contains the implementations of the ROS interface
code wrapped in objects. The launch files for each interfaced ROS node
still needs to be run BEFORE initializing these objects. In particular
roslaunch files:
    qualisys.launch
    qualisys_odom.launch model_name:=<blah blah blah>
need to be run. It is recommended you simply add these launch files to
whatever project launch file you are using.

TODO:
    - Add event-based functionality

Author - Frank J Jiang <frankji@kth.se>
"""

import sys
import os
import numpy as np
from threading import Thread

import rospy
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from math import sqrt

from models.bicycle_simple import SimpleBicycleState

class State(object):
    """
    Class representing the state of a vehicle.

    :param x: (float) x-coordinate
    :param y: (float) y-coordinate
    :param yaw: (float) yaw angle
    :param v: (float) speed
    """

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        """Instantiate the object."""
        super(State, self).__init__()
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

class QualisysOdom():

    def __init__(self, qualisys_model_name):
        self.qualisys_model_name = qualisys_model_name
        # rospy.init_node(self.qualisys_model_name + '_qualisys_odom')

        self.state = State()
        self.last_time = None

    def start(self):
        Thread(target=self._init_and_spin_ros, args=()).start()
        return self

    def _init_and_spin_ros(self):
        rospy.loginfo("Starting Qualisys Odometry Interface Node: \n"
                      + str(self))
        self.node_name = self.qualisys_model_name + '_qualisys_odom'
        self._collect_srvs()
        self._start_listen()

    def _collect_srvs(self):
        # rospy.wait_for_service('set_pose')
        # self.set_pose = rospy.ServiceProxy('set_pose', SetPose)
        pass

    def _start_listen(self):
        rospy.Subscriber(self.qualisys_model_name + '/odom', Odometry,
                         self._read_qualisys_odom_msg)
        rospy.loginfo("Qualisys Odometry Interface successfully initialized")
        rospy.spin()

    def _read_qualisys_odom_msg(self, msg):
        pose = msg.pose.pose.position
        vel = msg.twist.twist.linear

        q = msg.pose.pose.orientation
        quaternion = (q.x, q.y, q.z, q.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        yaw = euler[2]

        self.state.x = pose.x
        self.state.y = pose.y
        self.state.yaw = yaw
        self.state.v = sqrt(vel.x**2 + vel.y**2)

        self.last_time = rospy.get_time()

    def __repr__(self):
        return ""
    def __str__(self):
        return ""

    # def set_pose(self, qualisys_model_name, pose_to_set):
        # try:
            # self.set_pose(qualisys_model_name, pose_to_set)
        # except rospy.ServiceException as exc:
            # print(self.node_name + ": Set Pose service failed: " + str(exc))

    def is_publishing(self):

        if self.last_time is not None:
            is_publishing = (rospy.get_time() - self.last_time) < 0.2
            return is_publishing

        return False

    def get_state_obj(self):
        """Returns state object with variables state.x, state.y, state.yaw,
        state.v
        """
        return self.state

    def get_state(self):
        """Returns state as a list"""
        return [self.state.x, self.state.y, self.state.yaw, self.state.v]

    def get_state_np(self):
        """Returns state as a numpy array"""
        return np.array(self.get_state)


class QualisysSimpleOdom(QualisysOdom):

    def __init__(self, qualisys_model_name):
        QualisysOdom.__init__(self, qualisys_model_name)
        self.state = SimpleBicycleState()
