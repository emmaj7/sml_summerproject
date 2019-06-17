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
from controllers.control_interfaces import ControlInterface

from geometry_msgs.msg import Twist


## SIMULATION PARAMS ##########################################################
vehicle_name = "SVEA0"
init_state = [0, 0, 0, 0] #[x, y, yaw, v], units: [m, m, rad, m/s]
dt = 0.01

# animate results
show_animation = True
###############################################################################


# global variables for controlling simualtion with arrow keys
steering = 0
velocity = 0
def update_key_teleop(key_msg):
    global steering, velocity
    steering = math.radians(key_msg.angular.z)
    velocity = key_msg.linear.x


def main():

    rospy.init_node('SVEA_sim_keyboard')

    # initialize simulated model and control interface
    simple_bicycle_model = SimpleBicycleState(*init_state, dt=dt)
    ctrl_interface = ControlInterface(vehicle_name).start()
    rospy.sleep(0.5)

    # start background simulation thread
    simulator = SimSVEA(vehicle_name, simple_bicycle_model, dt, is_publish=True)
    simulator.start()
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

        # log data
        x.append(simple_bicycle_model.x)
        y.append(simple_bicycle_model.y)
        yaw.append(simple_bicycle_model.yaw)
        v.append(simple_bicycle_model.v)

        # update for animation
        if show_animation:
            plt.cla()
            plt.plot(x, y, "-b", label="trajectory")
            plot_car(simple_bicycle_model.x,
                     simple_bicycle_model.y,
                     simple_bicycle_model.yaw,
                     steering)
            plt.axis("equal")
            plt.grid(True)
            plt.title("Heading[deg]: "
                      + str(round(math.degrees(simple_bicycle_model.yaw), 4))
                      +" | Speed[m/s]:"
                      + str(round(simple_bicycle_model.v, 4)))
            plt.pause(0.001)

        # sleep so loop runs at 30Hz
        r.sleep()


if __name__ == '__main__':
    main()
