#!/usr/bin/env python

# Written by: Mikael Glamheden
# Last edited: 2019-07-17

import rospy
import tf
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from threading import Thread

class PoseSubscriber():
    '''Subscribes to the /zed/zed_node/pose topic. Saves the relevant parameters'''
    def __init__(self):
        self.node_name = 'zed_pose'
        self.x = None
        self.y = None
        self.yaw = None
        self.time = None

    def __repr__(self):
        return self.node_name
    def __str__(self):
        return self.node_name

    def get_state(self):
        '''Returns the state as a vector'''
        return [self.time, self.x, self.y, self.yaw]

    def start(self):
        Thread(target=self._init_and_spin_ros, args=()).start()
        return self

    def _init_and_spin_ros(self):
        rospy.loginfo(
                'Starting zed Pose Subscriber Node: \n'
                + str(self) + '_subscriber')
        self._start_listen()
        self._start_publish()
        rospy.spin()
    def _start_publish(self):
        self.pose_pub = rospy.Publisher(self.node_name+'/as_string',
                                                String,
                                                queue_size = 1)
    def _start_listen(self):
        rospy.Subscriber('/zed/zed_node/pose', PoseStamped , self._callback)
        rospy.loginfo("Controller Interface successfully initialized")

    def _callback(self, msg):
        self._update_vehicle_pose(msg)

    def _update_vehicle_pose(self, msg):
        '''Fetches the 2D parameters from the amcl_pose msg and
        stores them in object variables. Also publishes the 2D data.'''
        self.time = rospy.Time(msg.header.stamp.secs, msg.header.stamp.nsecs).to_sec()
        pose = msg.pose
        quat = [pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w]
        angles = tf.transformations.euler_from_quaternion(quat, axes='sxyz')
        yaw = angles[-1]
        pos = pose.position
        self.x = pos.x
        self.y = pos.y
        self.yaw = yaw
        message = ('Aquired amcl pose : ' +
                  'Timestamp: ' + str(self.time) + ', ' +
                  'x: ' + str(pos.x) + ', ' +
                  'y: ' + str(pos.y) + ', ' +
                  'yaw: ' + str(yaw))
        self.pose_pub.publish(message)
