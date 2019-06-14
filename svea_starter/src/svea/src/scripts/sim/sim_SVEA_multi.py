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

dirname = os.path.dirname(__file__)
pure_pursuit = os.path.join(dirname,
        '../../PythonRobotics/PathTracking/pure_pursuit/')
sys.path.append(os.path.abspath(pure_pursuit))

import pure_pursuit


## SIMULATION PARAMS ##########################################################
vehicle_name0 = "SVEA0"
vehicle_color0 = "blue"
init_state0 = [0, 0, 0, 0] # [x, y, yaw, v], units: [m, m, rad, m/s]

vehicle_name1 = "SVEA1"
vehicle_color1 = "green"
init_state1 = [-2, 0, 0, 0] # [x, y, yaw, v], units: [m, m, rad, m/s]

target_speed = 0.6 # [m/s]
dt = 0.01

# trajectory
cx = np.arange(0, 5, 0.1)
cy = [math.sin(ix) * ix for ix in cx]

# animate results?
show_animation = True
###############################################################################


## PURE PURSUIT PARAMS ########################################################
pure_pursuit.k = 0.4  # look forward gain
pure_pursuit.Lfc = 0.4  # look-ahead distance
pure_pursuit.L = 0.324  # [m] wheel base of vehicle
###############################################################################


def log_data(log, model, time):

    log["x"].append(model.x)
    log["y"].append(model.y)
    log["yaw"].append(model.yaw)
    log["v"].append(model.v)
    log["t"].append(time)

    return log


def main():

    rospy.init_node('SVEA_sim_multi')

    # initialize simulated model and control interface
    simple_bicycle_model0 = SimpleBicycleState(*init_state0, dt=dt)
    ctrl_interface0 = ControlInterface(vehicle_name0).start()

    simple_bicycle_model1 = SimpleBicycleState(*init_state1, dt=dt)
    ctrl_interface1 = ControlInterface(vehicle_name1).start()

    rospy.sleep(0.5)

    # start background simulation thread
    sim_car0 = SimSVEA(vehicle_name0, simple_bicycle_model0, dt, is_publish=True)
    sim_car1 = SimSVEA(vehicle_name1, simple_bicycle_model1, dt, is_publish=True)

    sim_car0.start()
    sim_car1.start()

    rospy.sleep(0.5)

    # log data
    log0 = {"x": [], "y": [], "yaw": [], "v": [], "t": []}
    log1 = {"x": [], "y": [], "yaw": [], "v": [], "t": []}

    # pure pursuit variables
    lastIndex = len(cx) - 1
    target_ind0 = pure_pursuit.calc_target_index(simple_bicycle_model0, cx, cy)
    target_ind1 = pure_pursuit.calc_target_index(simple_bicycle_model1, cx, cy)

    # simualtion + animation loop
    time = 0.0
    while lastIndex > target_ind0 and not rospy.is_shutdown():

        # compute control input via pure pursuit
        steering0, target_ind0 = \
            pure_pursuit.pure_pursuit_control(simple_bicycle_model0, cx, cy, target_ind0)
        ctrl_interface0.send_control(steering0, target_speed)

        steering1, target_ind1 = \
            pure_pursuit.pure_pursuit_control(simple_bicycle_model1, cx, cy, target_ind1)
        ctrl_interface1.send_control(steering1, target_speed)

        # log data
        log0 = log_data(log0, simple_bicycle_model0, time)
        log1 = log_data(log1, simple_bicycle_model1, time)

        # update for animation
        if show_animation:
            to_plot0 = (simple_bicycle_model0,
                        log0["x"], log0["y"],
                        steering0, vehicle_color0,
                        target_ind0)
            to_plot1 = (simple_bicycle_model1,
                        log1["x"], log1["y"],
                        steering1, vehicle_color1,
                        target_ind1)

            animate_multi([to_plot0, to_plot1])
        else:
            rospy.loginfo_throttle(1.5, simple_bicycle_model0)
            rospy.loginfo_throttle(1.5, simple_bicycle_model1)

        time += dt

    if show_animation:
        plt.close()
        to_plot0 = (simple_bicycle_model0,
                    log0["x"], log0["y"],
                    steering0, vehicle_color0,
                    target_ind0)
        to_plot1 = (simple_bicycle_model1,
                    log1["x"], log1["y"],
                    steering1, vehicle_color1,
                    target_ind1)
        animate_multi([to_plot0, to_plot1])
        plt.show()
    else:
        # just show resulting plot if not animating
        to_plot0 = (simple_bicycle_model0,
                    log0["x"], log0["y"],
                    vehicle_color0)
        to_plot1 = (simple_bicycle_model1,
                    log1["x"], log1["y"],
                    vehicle_color1)
        plot_trajectory(*to_plot0)
        plot_trajectory(*to_plot1)
        plt.show()


def plot_trajectory(car_model, x, y, traj_color="-b"):
    plt.plot(cx, cy, ".r", label="course")
    plt.plot(x, y, traj_color, label="trajectory")
    plt.axis("equal")
    plt.grid(True)

def animate_multi(to_plot_list):
    plt.cla()
    for to_plot in to_plot_list:
        animate_pure_pursuit(*to_plot)
    plt.pause(0.001)

def animate_pure_pursuit(car_model, x, y, steering, color, target_ind):
    """ Single animation update for pure pursuit."""
    plot_trajectory(car_model, x, y, color)
    # plot pure pursuit current target
    plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
    # plot car current state
    plot_car(car_model.x,
             car_model.y,
             car_model.yaw,
             steering,
             color)

if __name__ == '__main__':
    main()
