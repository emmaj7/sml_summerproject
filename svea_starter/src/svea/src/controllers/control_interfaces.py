#!/usr/bin/env python

"""
ROS interface objects for sending control to a SVEA car.

This module simply contains the implementations of the ROS interface
code wrapped in objects. The launch files for each interfaced ROS node
still needs to be run BEFORE initializing these objects, unless the
code is being run with a simulator object. In particular, the required
roslaunch file:
    rosserial_python
needs to be run.

It is recommended you simply add these launch files to whatever project
launch file you are using.

Author - Frank J Jiang <frankji@kth.se>
"""


from threading import Thread
from math import radians, degrees

import rospy
from geometry_msgs.msg import Twist
# from low_level_interface.msg import lli_ctrl_request, lli_ctrl_actuated
from svea_arduino.msg import lli_ctrl

class ControlInterface():

    OPERATING_FREQ = 30 # [Hz]

    MAX_STEER_PERCENT = 80 # [%]
    MAX_VELOCITY_PERCENT = 100 #[%]

    MAX_VELOCITY = 1 #[m/s]

    def __init__(self, vehicle_name=""):
        # rospy.init_node(vehicle_name + '_control_interface')

        self.vehicle_name = vehicle_name

        # self.ctrl_request = lli_ctrl_request()

        self.ctrl_request = lli_ctrl()

        self.last_ctrl_update = rospy.get_time()

        self.is_stop = False
        self.is_emergency = False

        # log of control requests and control actuated
        self.ctrl_request_log = []
        self.ctrl_actuated_log = []

    def start(self):
        Thread(target=self._init_and_spin_ros, args=()).start()
        return self

    def _init_and_spin_ros(self):
        rospy.loginfo(
                'Starting Controller Interface Node: \n'
                + str(self))
        self.node_name = self.vehicle_name + '_control_interface'

        self._start_publish()
        self._start_listen()
        rospy.spin()

    def _start_listen(self):
        rospy.Subscriber(self.vehicle_name+'/lli/ctrl_actuated', lli_ctrl,
                         self._read_ctrl_actuated)
        rospy.loginfo("Controller Interface successfully initialized")

    def _start_publish(self):
        self.ctrl_request_pub = rospy.Publisher(self.vehicle_name+'/lli/ctrl_request',
                                                lli_ctrl,
                                                queue_size = 1)

    def _read_ctrl_actuated(self, msg):
        self.ctrl_actuated_log.append(msg)

    def _build_param_printout(self):
        # collect important params
        steering = self.ctrl_request.steering
        velocity = self.ctrl_request.velocity
        trans_diff = self.ctrl_request.trans_diff
        # differential_front = self.ctrl_request.differential_front
        # differential_rear = self.ctrl_request.differential_rear
        ctrl = self.ctrl_request.ctrl

        return ("## Vehicle: {0}\n".format(self.vehicle_name)
                +"  -ctrl request:\n"
                +"      steering   - {0}\n".format(steering)
                +"      velocity   - {0}\n".format(velocity)
                +"      trans      - {0}\n".format(trans_diff)
                +"      ctrl_code  - {0}\n".format(ctrl)
                +"  -Is stopped: {0}\n".format(self.is_stop)
                +"  -Is emergency: {0}\n".format(self.is_emergency))

    def __repr__(self):
        return self._build_param_printout()
    def __str__(self):
        return self._build_param_printout()

    def _steer_to_percent(self, steering):
        steering = float(steering)
        if degrees(steering) >= 0:
            steer_PWM = (1495 - 5.016 * degrees(steering)
                         - 0.2384 * degrees(steering)**2)
            steer_percent = (steer_PWM-1500)/600 * 100
        else:
            steer_PWM = (1495 - 5.016 * degrees(steering)
                          + 0.2384 * degrees(steering)**2)
            steer_percent = (steer_PWM-1500)/600 * 100

        steer_percent = -steer_percent # steering flipped

        return int(steer_percent)

    def _vel_to_percent(self, velocity):
        velocity = float(velocity)
        vel_percent = velocity/self.MAX_VELOCITY * 100
        return int(vel_percent)

    def _clip_ctrl(self, steer_percent, vel_percent):
        clipped_steering = min(self.MAX_STEER_PERCENT,
                               max(-self.MAX_STEER_PERCENT, steer_percent))
        clipped_velocity = min(self.MAX_VELOCITY_PERCENT,
                               max(-self.MAX_VELOCITY_PERCENT, vel_percent))

        return clipped_steering, clipped_velocity

    def send_control(self, steering, velocity,
                     transmission = 0,
                     differential_front = 0,
                     differential_rear = 0,
                     ctrl_code = 0):

        steer_percent = self._steer_to_percent(steering)
        vel_percent = self._vel_to_percent(velocity)

        steer_percent, vel_percent = self._clip_ctrl(steer_percent, vel_percent)

        # Range is changed from percent to: -127 - 127
        steer_percent = steer_percent * -1.27
        vel_percent = vel_percent * 1.27

        self.ctrl_request.steering = steer_percent
        self.ctrl_request.velocity = vel_percent
        self.ctrl_request.trans_diff = 8 # set to low gear value.
        self.ctrl_request.ctrl = ctrl_code

        if not self.is_emergency or not self.is_stop or not self.is_persistent:
            self.ctrl_request_pub.publish(self.ctrl_request)
            self.ctrl_request_log.append(self.ctrl_request)

    def set_is_stop(self, is_stop = True):
        self.is_stop = is_stop

    def set_is_emergency(self, is_emergency = True):
        self.is_emergency = is_emergency
        self.is_stop = is_emergency
        if is_emergency == True:
            rospy.loginfo("Blocking {0} for emergency".format(self.node_name))
        else:
            rospy.loginfo("Unblocking {0} after emergency".format(self.node_name))


class ControlInterfaceWTeleop(ControlInterface):
    """
    Inherits from control interface but layers functions for receiving
    and implementing remote teleop control from WebRTC app.

    percent_teleop serves as way to control how much control teleop has
    in a simple control input linear combination between teleop and
    automation.
    """

    def __init__(self, vehicle_name="", percent_teleop=1):
        ControlInterface.__init__(self, vehicle_name)

        self.teleop_cmd = Twist()

        self.percent_teleop = percent_teleop

    def _start_listen(self):
        rospy.Subscriber(self.vehicle_name+'/lli/ctrl_actuated', lli_ctrl_actuated,
                         self._read_ctrl_actuated)
        rospy.Subscriber('/teleop_cmd', Twist,
                         self._read_ctrl_teleop)
        rospy.loginfo(
                "Controller Interface successfully initialized")

    def _read_ctrl_teleop(self, msg):
        self.teleop_cmd = msg

    def _build_param_printout(self):
        # collect important params
        steering = self.ctrl_request.steering
        velocity = self.ctrl_request.velocity
        trans_diff = self.ctrl_request.trans_diff
        ctrl_code = self.ctrl_request.ctrl_code

        return ("## Vehicle: {0}\n".format(self.vehicle_name)
                +"  -ctrl request:\n"
                +"      steering   - {0}\n".format(steering)
                +"      velocity   - {0}\n".format(velocity)
                +"      trans      - {0}\n".format(trans_diff)
                +"      ctrl_code  - {0}\n".format(ctrl_code)
                +"  -% teleop: {0}\n".format(self.percent_teleop)
                +"  -Is stopped: {0}\n".format(self.is_stop)
                +"  -Is emergency: {0}\n".format(self.is_emergency))

    def combine(self, auto_steering, auto_velocity):
        teleop_steering = self.teleop_cmd.angular.z
        teleop_velocity = self.teleop_cmd.linear.x

        p = self.percent_teleop
        steering = p * teleop_steering + (1-p) * auto_steering
        velocity = p * teleop_velocity + (1-p) * auto_velocity

        return int(steering), int(velocity)

    def send_control(self, steering = 0, velocity = 0,
                     transmission = 0,
                     differential_front = 0,
                     differential_rear = 0,
                     ctrl_code = 0):
        """ Inputs to this function are automation control, not teleop """

        steer_percent = self._steer_to_percent(steering)
        vel_percent = self._vel_to_percent(velocity)

        # bound input
        steer_percent, vel_percent = self._clip_ctrl(steer_percent, vel_percent)
        # fuse automation and teleop inputs
        steer_percent, vel_percent = self.combine(steer_percent, vel_percent)

        self.ctrl_request.steering = steer_percent
        self.ctrl_request.velocity = vel_percent
        self.ctrl_request.trans_diff = 9
        self.ctrl_request.ctrl = ctrl_code

        if not self.is_emergency or not self.is_stop or not self.is_persistent:
            self.ctrl_request_pub.publish(self.ctrl_request)
            self.ctrl_request_log.append(self.ctrl_request)

    def set_ctrl_blend(percent_teleop):
        if percent_teleop >= 0 and percent_teleop <= 1:
            self.percent_teleop = percent_teleop
        else:
            rospy.loginfo("Value not between [0, 1]. Not setting control blend"
                          + "ratio")


class OldControlInterface():
    """
    Control interface compliant with old arduino firmware where low
    level control is set by raw PWM values from 900-2100.
    """

    MAX_STEER_ANGLE = radians(30) #radians
    MAX_VELOCITY = 1 #[m/s]

    def __init__(self, vehicle_name):

        self.vehicle_name = vehicle_name

        self.ctrl_request = Twist()
        self.brake_request = Twist()
        self.last_ctrl_update = rospy.get_time()

        self.is_stop = False
        self.is_emergency = False

        # log of control requests and control actuated
        self.ctrl_request_log = []
        self.ctrl_actuated_log = []

    def start(self):
        Thread(target=self._init_and_spin_ros, args=()).start()
        return self

    def _init_and_spin_ros(self):
        rospy.loginfo(
                'Starting Old Controller Interface Node: \n'
                + str(self))
        self.node_name = self.vehicle_name + '_old_control_interface'

        self._start_listen()
        self._start_publish()
        rospy.spin()

    def _start_listen(self):
        # nothing to subscribe to
        rospy.loginfo(
                "Old Controller Interface successfully initialized")

    def _start_publish(self):
        self.ctrl_request_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)

    def __repr__(self):
        # rospy.get_param('/blah')
        return "* <params should be here> *"
    def __str__(self):
        # rospy.get_param('/blah')
        return "* <params should be here> *"

    def brake(self):
        #TODO add timed braking
        self.brake_request.angular.z = self.ctrl_request.angular.z
        self.ctrl_request_pub.publish(self.brake_request)
        self.ctrl_request_log.append(self.brake_request)

    def send_PWM(self, steer_PWM, vel_PWM):
        self.ctrl_request.angular.z = steer_PWM
        self.ctrl_request.linear.x = vel_PWM

        if self.is_emergency or self.is_stop:
            self.brake()
        else:
            self.ctrl_request_pub.publish(self.ctrl_request)
            self.ctrl_request_log.append(self.ctrl_request)

    def _steer_to_PWM(self, steering):
        if degrees(steering) >= 0:
            steer_PWM = int(1495 - 5.016 * degrees(steering)
                            - 0.2384 * degrees(steering)**2)
        else:
            steer_PWM = int(1495 - 5.016 * degrees(steering)
                            + 0.2384 * degrees(steering)**2)
        return steer_PWM

    def _vel_to_PWM(self, velocity):
        vel_PWM = 1630
        return vel_PWM

    def _clip_ctrl(self, steering, velocity):
        clipped_steering = min(self.MAX_STEER_ANGLE,
                               max(-self.MAX_STEER_ANGLE, steering))
        #TODO add this in after finding velocity model
        clipped_velocity = min(self.MAX_VELOCITY,
                               max(-self.MAX_VELOCITY, steering))
        clipped_velocity = velocity

        return clipped_steering, clipped_velocity

    def send_control(self, steering, velocity,
                     gear, transmission = 0,
                     differential_front = 0,
                     differential_rear = 0,
                     ctrl_code = 0):

        steering, velocity = self._clip_ctrl(steering, velocity)

        steer_PWM = self._steer_to_PWM(steering)
        vel_PWM = self._vel_to_PWM(velocity)

        self.send_PWM(steer_PWM, vel_PWM)

    def set_is_stop(self, is_stop = True):
        self.is_stop = is_stop

    def set_is_emergency(self, is_emergency = True):
        self.is_emergency = is_emergency
        self.is_stop = is_emergency
        if is_emergency == True:
            rospy.loginfo("Blocking {0} for emergency".format(self.node_name))
        else:
            rospy.loginfo("Unblocking {0} after emergency".format(self.node_name))
