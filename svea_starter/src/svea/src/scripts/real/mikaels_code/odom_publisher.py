#!/usr/bin/env python

# High level interface to simulate the SVEA cars.
# Written by Mikael Glamheden
# Last updated: 2019-07-17


import math
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from threading import Thread

class OdomPublisher():
    '''Class for interface with publishing to the odometry topic.
        Given a 2D state, and steering information a geometry message is published.
        The transform between /odom and /base_link is also broadcasted.'''

    def __init__(self, vehicle_name=""):
        self.operating_freq = 30 # [Hz]
        self.vehicle_name = vehicle_name
        self.is_stop = False
        self.is_emergency = False
        self.odom_tf = tf.TransformBroadcaster()
        self.odom = Odometry()
        self.last_steering = 0
        self.odom_pub = None
        self.node_name = None

    def __repr__(self):
        return self.node_name
    def __str__(self):
        return self.node_name

    def start(self):
        Thread(target=self._init_and_spin_ros, args=()).start()
        return self
    def _init_and_spin_ros(self):
        self.node_name = self.vehicle_name + '_odom'
        rospy.loginfo(
                'Starting Odometry node: \n'
                + str(self))
        self._start_publish()
        print('started publisher')
        rospy.spin()
    def _start_publish(self):
        self.odom_pub = rospy.Publisher(self.vehicle_name + "/odom",
                                        Odometry,
                                        queue_size=1)

    def send_odometry(self, state, steering):
        """Publishes the position of the car as estimated by the SimpleBicycle
        model simulation + the velocity commands.
        state is list [x, y, yaw, velocity]"""

        # quaternion from yaw
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, state[2])

        # publish the transform over tf
        self.odom_tf.sendTransform((state[0], state[1], 0.),
                                    odom_quat,
                                    rospy.Time.now(),
                                    "base_link",
                                    "odom")
        # Set the time stamp
        self.odom.header.stamp = rospy.Time.now()
        # set the frame id
        self.odom.header.frame_id = self.vehicle_name + '_odom'
        # set the position
        self.odom.pose.pose = Pose(Point(state[0],
                                         state[1],
                                         0.0),
                                   Quaternion(*odom_quat)
        )
        # set the velocity
        vx = state[3]*math.cos(state[2])
        vy = state[3]*math.sin(state[2])
        vth = (steering - self.last_steering)/self.operating_freq # approximate the 2D yaw dot
        self.odom.child_frame_id = "base_link"
        self.odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))
        if not self.is_emergency or not self.is_stop or not self.is_persistent:
            self.odom_pub.publish(self.odom)

    def set_is_stop(self, is_stop = True):
        self.is_stop = is_stop

    def set_is_emergency(self, is_emergency = True):
        self.is_emergency = is_emergency
        self.is_stop = is_emergency
        if is_emergency == True:
            rospy.loginfo("Blocking {0} for emergency".format(self.node_name))
        else:
            rospy.loginfo("Unblocking {0} after emergency".format(self.node_name))
