#!/usr/bin/env python

import sys
import os
import rospy
import math
import numpy as np
import matplotlib.pyplot as plt

dirname = os.path.dirname(__file__)
svea = os.path.join(dirname, '../../')
sys.path.append(os.path.abspath(svea))

from models.bicycle_simple import SimpleBicycleState, plot_car
from simulators.sim_SVEA_simple import SimSVEA
from controllers.control_interfaces import ControlInterface
from localizers.qualisys_localizers import QualisysSimpleOdom

dirname = os.path.dirname(__file__)
pure_pursuit = os.path.join(dirname,
        '../../PythonRobotics/PathTracking/pure_pursuit/')
sys.path.append(os.path.abspath(pure_pursuit))

import pure_pursuit


## SIMULATION PARAMS ##########################################################
vehicle_name = "SVEA5"
init_state = [0, 0, 0, 0] # [x, y, yaw, v], units: [m, m, rad, m/s]
target_speed = 0.4 # [m/s]
dt = 0.01

# trajectory
cx = np.arange(0, 2, 0.1)
cy = [math.sin(2*ix) for ix in cx]

# animate results?
show_animation = False
###############################################################################


## PURE PURSUIT PARAMS ########################################################
pure_pursuit.k = 0.4  # look forward gain
pure_pursuit.Lfc = 0.4  # look-ahead distance
pure_pursuit.L = 0.324  # [m] wheel base of vehicle
###############################################################################
qualisys_model_name = "SVEA5"

def main():

    rospy.init_node('SVEA_purepursuit')

    # initialize simulated model and control interface
    qualisys_odom = QualisysSimpleOdom(qualisys_model_name).start()
    ctrl_interface = ControlInterface().start()
    rospy.sleep(0.5)


    # log data
    x = []
    y = []
    yaw = []
    v = []
    t = []
    # wait to initialize when qualisys starts
    while not qualisys_odom.is_publishing() and not rospy.is_shutdown():

        if not qualisys_odom.last_time is None:
            qualisys_msg_delay = rospy.get_time()-qualisys_odom.last_time
        else:
            qualisys_msg_delay = "n/a"

        rospy.loginfo_throttle(2, "Waiting for Qualisys Odometry to become "
                                  +"available: {0} s".format(qualisys_msg_delay))
        pass # wait until qualisys data available

    rospy.loginfo("Starting Pure Pursuit")
    start_time = rospy.get_time()

    # pure pursuit variables
    lastIndex = len(cx) - 1
    curr_state = qualisys_odom.get_state_obj()
    target_ind = pure_pursuit.calc_target_index(curr_state, cx, cy)

    # control loop
    r = rospy.Rate(30) #SVEA cars operate on 30Hz!
    time = 0.0
    while lastIndex > target_ind and not rospy.is_shutdown():

        time = rospy.get_time() - start_time

        if qualisys_odom.is_publishing():
            # localize
            state = qualisys_odom.get_state_obj()

            # compute control input via pure pursuit
            steering, target_ind = \
                pure_pursuit.pure_pursuit_control(state, cx, cy, target_ind)
            ctrl_interface.send_control(steering, target_speed)

            # log data
            x.append(state.x)
            y.append(state.y)
            yaw.append(state.yaw)
            v.append(state.v)
            t.append(time)
	else:
            qualisys_msg_delay = rospy.get_time()-qualisys_odom.last_time

            rospy.loginfo_throttle(2, "No Qualisys Odometry available. "
                                      +"{0} s".format(qualisys_msg_delay))
        # update for animation
        if show_animation:
            to_plot = (state,
                       x, y,
                       steering, target_ind)
            animate_pure_pursuit(*to_plot)
        else:
            rospy.loginfo_throttle(1.5, state)

        r.sleep()


    if len(x) > 0:
        rospy.loginfo("Finished Trajectory.")
        if show_animation:
            plt.close()
            to_plot = (state,
                       x, y,
                       steering, target_ind)
            animate_pure_pursuit(*to_plot)
            plt.show()
        else:
            # just show resulting plot if not animating
            to_plot = (state, x, y)
            plot_trajectory(*to_plot)
            plt.show()


def plot_trajectory(car_model, x, y):
    plt.plot(cx, cy, ".r", label="course")
    plt.plot(x, y, "-b", label="trajectory")
    plt.axis("equal")
    plt.grid(True)
    plt.title("Heading[deg]: "
                + str(math.degrees(car_model.yaw))[:4]
                +" | Speed[m/s]:"
                + str(car_model.v)[:4])

def animate_pure_pursuit(car_model, x, y, steering, target_ind):
    """ Single animation update for pure pursuit."""
    plt.cla() #clear last plot/frame
    plot_trajectory(car_model, x, y)
    # plot pure pursuit current target
    plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
    # plot car current state
    plot_car(car_model.x,
             car_model.y,
             car_model.yaw,
             steering)
    plt.pause(0.001)

if __name__ == '__main__':
    main()
