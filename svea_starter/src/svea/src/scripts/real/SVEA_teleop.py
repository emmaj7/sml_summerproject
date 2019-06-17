#!/usr/bin/env python

import sys
import os
import rospy
import math
import matplotlib.pyplot as plt

dirname = os.path.dirname(__file__)
svea = os.path.join(dirname, '../../')
sys.path.append(os.path.abspath(svea))

from models.bicycle_simple import SimpleBicycleState, plot_car
from simulators.sim_SVEA_simple import SimSVEA
from controllers.control_interfaces import ControlInterfaceWTeleop
from localizers.qualisys_localizers import QualisysSimpleOdom

from geometry_msgs.msg import Twist


##  PARAMS ##########################################################
vehicle_name = "SVEA0"
init_state = [0, 0, 0, 0] #[x, y, yaw, v], units: [m, m, rad, m/s]
dt = 0.01

# animate results
show_animation = False
###############################################################################

def main():

    rospy.init_node('SVEA_teleop')

    # trigger controller to send teleop commands to car
    ctrl_interface = ControlInterfaceWTeleop().start()
    rospy.sleep(0.5)

    # log data
    x = []
    y = []
    yaw = []
    v = []

    # simualtion + animation loop
    r = rospy.Rate(30) #SVEA cars operate on 30Hz!
    while not rospy.is_shutdown():

        # trigger controller to send teleop commands to car
        ctrl_interface.send_control()

        # log data
	curr_state = qualisys_odom.get_state_obj()
        x.append(curr_state.x)
        y.append(curr_state.y)
        yaw.append(curr_state.yaw)
        v.append(curr_state.v)

        # update for animation
        if show_animation:
            steer_cmd = math.radians(ctrl_interface.teleop_cmd.angular.z/100 * 30)

            plt.cla()
            plt.plot(x, y, "-b", label="trajectory")
            plot_car(curr_state.x,
                     curr_state.y,
                     curr_state.yaw,
                     steer_cmd)
            plt.axis("equal")
            plt.grid(True)
            plt.title("Heading[deg]: "
                      + str(round(math.degrees(curr_state.yaw), 4))
                      +" | Speed[m/s]:"
                      + str(round(curr_state.v, 4)))
            plt.pause(0.001)

        # sleep so loop runs at 30Hz
        r.sleep()


if __name__ == '__main__':
    main()
