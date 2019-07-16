import math

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3


class OdomPublisher():
    OPERATING_FREQ = 30 # [Hz]

    def __init__(self, vehicle_name=""):
        self.vehicle_name = vehicle_name
        self.current_time = rospy.get_time()
        self.is_stop = False
        self.is_emergency = False
        self.odom_tf = tf.TransformBroadcaster()
        self.odom = Odometry()
        self.last_steering = 0

    def start(self):
        Thread(target=self._init_and_spin_ros, args=()).start()
        return self
    def _init_and_spin_ros(self):
        rospy.loginfo(
                'Starting Odometry node: \n'
                + str(self))
        self.node_name = self.vehicle_name + '_odom'

        self._start_publish()
        rospy.spin()
    def _start_publish(self):
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=20)
    def send_odometry(self, state, steering):
        """Publishes the position of the car as estimated by the SimpleBicycle
        model simulation + the velocity commands.
        pose_estimate is list [x, y, yaw, velocity]"""

        # since all odometry is 6DOF we'll need a quaternion created from yaw
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, pose_estimate[2])
            # first, we'll publish the transform over tf
        self.odom_tf.sendTransform(
            (pose_estimate[0], pose_estimate[1], 0.),
            odom_quat,
            self.current_time,
            "base_link",
            "odom"
        )
        self.odom.header.stamp = self.current_time
        self.odom.header.frame_id = 'id'
        # set the position
        self.odom.pose.pose = Pose(Point(pose_estimate[0],
                                         pose_estimate[1],
                                         0.0),
                                   Quaternion(*odom_quat)
        )

        # set the velocity
        vx = state[3]*math.cos(yaw)
        vy = state[3]*math.sin(yaw)
        vth = steering - self.last_steering
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
