#!/usr/bin/env python

import sys
import os
import rospy
import math
import matplotlib.pyplot as plt

dirname = os.path.dirname(__file__)
svea = os.path.join(dirname, '../../')
sys.path.append(os.path.abspath(svea))

from models.bicycle_simple import plot_car
from controllers.control_interfaces import ControlInterface
from localizers.qualisys_localizers import QualisysSimpleOdom

from geometry_msgs.msg import Twist


## PARAMS #####################################################################
qualisys_model_name = "/SVEA0"

# animate results
show_animation = True
###############################################################################


# global variables for controlling simulation with arrow keys
steering = 0
velocity = 0
def update_key_teleop(key_msg):
    global steering, velocity
    steering = math.radians(key_msg.angular.z)
    velocity = key_msg.linear.x


def main():

    rospy.init_node('SVEA_keyboard')

    # initialize simulated model and control interface
    qualisys_odom = QualisysSimpleOdom(qualisys_model_name).start()
    ctrl_interface = ControlInterface().start()
    rospy.sleep(0.5)

    # log data
    x = []
    y = []
    yaw = []
    v = []

    # hook callback for updating keyboard commands
    rospy.Subscriber("/key_vel", Twist, update_key_teleop)

    # simualtion + animation loop
    r = rospy.Rate(30) #SVEA cars operate on 30Hz!
    while not rospy.is_shutdown():

        # pass keyboard commands to ctrl interface
        ctrl_interface.send_control(steering, velocity)

        if qualisys_odom.is_publishing():

            state = qualisys_odom.get_state()

            # log data
            x.append(state.x)
            y.append(state.y)
            yaw.append(state.yaw)
            v.append(state.v)

        # sleep so loop runs at 30Hz
        r.sleep()


if __name__ == '__main__':
    main()
