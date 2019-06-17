#!/usr/bin/env python

"""
LIDAR module that contains ROS interface objects for the SML-used LIDARs

This module simply contains the implementations of the ROS interface
code wrapped in objects. The launch files for each interfaced ROS node
still needs to be run BEFORE initializing these objects. The first
object is a generic LIDAR superclass to be inherited by each of the
SML-used/implemented LIDARs.

SML-used/implemented LIDARs:
    - Hokuyo Lidar UST-10LX
    - RPLidar A2/A3

TODO:
    - Add velodyne interface

Author - Frank J Jiang <frankji@kth.se>
"""

from threading import Thread

import rospy
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty


class RPLidar():

    # options: serial_port, serial_baudrate, frame_id, inverted,
    #          angle_compensate scan_mode
    ROS_PARAM_PREFIX = "rplidarNode"
    IMPORTANT_ROS_PARAMS = ["serial_port", "frame_id",
                            "angle_compensate"]

    def __init__(self, emergency_dist=0.2):
        # rospy.init_node('rplidar_handler')
        self.node_name = "RPLidar A2/A3 Handler"

        self.scan = []

        self.emergency_dist = emergency_dist
        self.is_emergency = False

    def start(self):
        Thread(target=self._init_and_spin_ros, args=()).start()
        return self

    def _init_and_spin_ros(self):
        rospy.loginfo("Starting Lidar Interface Node: \n" + str(self))
        self._collect_srvs()
        self._start_listen()

    def _collect_srvs(self):
        rospy.wait_for_service('start_motor')
        self.start_motor_srv = rospy.ServiceProxy('start_motor', Empty)

        rospy.wait_for_service('stop_motor')
        self.stop_motor_srv = rospy.ServiceProxy('stop_motor', Empty)

    def _start_listen(self):
        rospy.Subscriber('scan', LaserScan, self._read_scan)
        rospy.loginfo("Lidar Interface successfully initialized")
        rospy.spin()

    def _read_scan(self, scan_msg):
        self.scan = scan_msg.ranges

        self.angle_min = scan_msg.angle_min
        self.angle_max = scan_msg.angle_max
        self.angle_increment = scan_msg.angle_increment

        self.time_increment = scan_msg.time_increment

        self.last_scan_time = scan_msg.scan_time

        self.is_emergency = min(self.scan) < self.emergency_dist

    def _build_param_printout(self):
        param_str = "{0}:\n".format(self.node_name)

        for param_name in self.IMPORTANT_ROS_PARAMS:
            curr_param = rospy.get_param(self.ROS_PARAM_PREFIX+ '/' + param_name)
            param_str += "  - {0}: {1}\n".format(param_name, curr_param)

        return param_str

    def __repr__(self):
        return self._build_param_printout()
    def __str__(self):
        return self._build_param_printout()

    def start_motor(self):
        try:
            self.start_motor_srv()
        except rospy.ServiceException as exc:
            print(self.node_name + ": Start motor service failed: " + str(exc))

    def stop_motor(self):
        try:
            self.stop_motor_srv()
        except rospy.ServiceException as exc:
            print(self.node_name + ": Stop motor service failed: " + str(exc))

    def get_raw_scan(self):
        return self.scan

    def get_raw_scan_with_time(self):
        return self.last_scan_time, self.scan

    def get_is_emergency(self):
        return self.is_emergency
