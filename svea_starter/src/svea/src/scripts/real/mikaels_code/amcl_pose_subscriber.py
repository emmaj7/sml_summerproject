#!/usr/bin/env python
import rospy
import tf
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from threading import Thread

class AmclPoseSubscriber():
    def __init__(self, vehicle_model):
        self.vehicle_model = vehicle_model
        self.node_name = 'amcl_pose_subscriber'
    def __repr__(self):
        return self.node_name
    def __str__(self):
        return self.node_name

    def start(self):
        Thread(target=self._init_and_spin_ros, args=()).start()
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped , self._callback)
        return self
    def _init_and_spin_ros(self):
        rospy.loginfo(
                'Starting amcl Pose Subscriber Node: \n'
                + str(self))
        self._start_listen()
        rospy.spin()
    def _start_listen(self):
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped , self._callback)
        rospy.loginfo("Controller Interface successfully initialized")

    def _callback(self, msg):
        rospy.loginfo(rospy.get_caller_id() + '\n %s', msg)
        self._update_vehicle_pose(msg)

    def _update_vehicle_pose(self, msg):
        pose = msg.pose.pose
        quat = [pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w]
        angles = tf.transformations.euler_from_quaternion(quat, axes='sxyz')
        yaw = angles[-1]
        pos = pose.position
        self.vehicle_model.correct_state(pos.x, pos.y, yaw)
