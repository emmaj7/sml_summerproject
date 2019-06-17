#!/usr/bin/env python
# Written by Mikael Glamheden
# 2019-06-04

import sys
import os
import rospy
import math
import numpy as np
import matplotlib.pyplot as plt

import mikaels_code.line_follower as lf
import mikaels_code.car_commands as cc
import mikaels_code.data_log as dlog

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

dirname = os.path.dirname(__file__)
qualisys_localizers = os.path.join(dirname,
        '../../localizers/')
sys.path.append(os.path.abspath(qualisys_localizers))
import qualisys_localizers as ql

class CarHighLevelCommands():
    """Class with high level methods intended to be called from a blockly-generated code."""
    def __init__(self, simulation = True, vehicle_name = "SVEA0",
                       qualisys_model_name = "SVEA0",
                       init_state = [0, 0, 0.2, 0],
                       target_speed = 0.6,
                       dt = 0.01):
        self.simulation = simulation
        if self.simulation:
            # initialize simulated model and control interface
            self.vehicle_model = SimpleBicycleState(*init_state, dt=dt)
            self.ctrl_interface = ControlInterface(vehicle_name).start()
            rospy.sleep(0.5)
            # start background simulation thread
            self.simulator = SimSVEA(vehicle_name, self.vehicle_model, dt, is_publish=True)
            self.simulator.start()
            rospy.sleep(0.5)
        else:
            # initialize odometry and control interface for real car
            self.vehicle_model = ql.QualisysSimpleOdom(qualisys_model_name).start()
            self.ctrl_interface = ControlInterface().start()
            rospy.sleep(0.5)

        self.target_speed = target_speed # [m/s]
        self.target_state = init_state # x, y, yaw, v
        # object to log data in
        self.data_log = dlog.Data_log()

    def _plot_trajectory(self, cx, cy):
        x = self.data_log.get_x()
        y = self.data_log.get_y()
        plt.plot(cx, cy, ".r", label="course")
        plt.plot(x, y, "-b", label="trajectory")
        plt.axis("equal")
        plt.grid(True)
        state = self.vehicle_model.get_state()
        plt.title("Heading[deg]: "
                    + str(math.degrees(state[2]))[:4]
                    +" | Speed[m/s]:"
                    + str(state[3])[:4])

    def _animate_pure_pursuit(self, steering, cx, cy, target_ind):
        """ Single animation update for pure pursuit."""
        plt.cla() #clear last plot/frame
        self._plot_trajectory(cx, cy)
        # plot pure pursuit current target
        plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
        # plot car current state
        state = self.vehicle_model.get_state()
        plt.title("Heading[deg]: "
                    + str(math.degrees(state[2]))[:4]
                    +" | Speed[m/s]:"
                    + str(state[3])[:4])
        plot_car(state[0],
                 state[1],
                 state[2],
                 steering)
        plt.pause(0.001)

    def _animate_robot_path(self,steering,x0,y0,xg,yg):
        plt.cla()
        x = self.data_log.get_x()
        y = self.data_log.get_y()
        plt.plot([x0, xg],[y0, yg],linestyle='--',label='intended trajectory')
        plt.plot(x, y, "-b", label="trajectory")
        plt.axis("equal")
        plt.grid(True)
        # plot car current state
        state = self.vehicle_model.get_state()
        plt.title("Heading[deg]: "
                    + str(math.degrees(state[2]))[:4]
                    +" | Speed[m/s]:"
                    + str(state[3])[:4])
        plot_car(state[0],
                 state[1],
                 state[2],
                 steering)
        plt.pause(0.001)

    def _turn(self, angle):
        """Turn the car a given number of degrees relative to current orientation."""
        l = 1
        tol1 = 1.1*math.pi/180 # angular tolerance
        tol2 = 2 # positional tolerance
        x0 = self.target_state[0]
        y0 = self.target_state[1]
        xg = x0 + l*math.cos(angle)
        yg = y0 + l*math.sin(angle)
        yaw_ref = math.atan2(yg-y0,xg-x0)
        yaw_ref = lf.normalize_angle(yaw_ref)
        at_goal = False
        while not at_goal and not rospy.is_shutdown():
            state = self.vehicle_model.get_state()
            x = state[0]
            y = state[1]
            yaw = state[2]
            # calculate and send control to car
            velocity, steering = lf.orientation_controller(x, y, yaw, x0, y0, xg, yg)
            self.ctrl_interface.send_control(steering,velocity)

            # log data
            data = tuple(self.vehicle_model.get_state())
            self.data_log.append_data(*data)
            if self.simulation:
                self._animate_robot_path(steering, x0, y0, xg, yg)

            # Done if angle is close enough
            dist = np.linalg.norm([x0-x,y0-y])
            if abs(yaw_ref-yaw) < tol1 and dist < tol2:
                at_goal = True
        self.target_state[0] = x
        self.target_state[1] = y
        print('Completed turn!')

    def pure_pursuit(self, cx, cy, show_animation = True):
        """Make car drive along given trajectory. Uses pure pursuit algorithm."""
        # Pure pursuit params
        pure_pursuit.k = 0.4  # look forward gain
        pure_pursuit.Lfc = 0.4  # look-ahead distance
        pure_pursuit.L = 0.324  # [m] wheel base of vehicle

        # pure pursuit calculation
        lastIndex = len(cx) - 1
        target_ind = pure_pursuit.calc_target_index(self.vehicle_model, cx, cy)

        # simualtion + animation loop
        while lastIndex > target_ind and not rospy.is_shutdown():
            # compute control input via pure pursuit
            steering, target_ind = \
                pure_pursuit.pure_pursuit_control(self.vehicle_model, cx, cy, target_ind)
            self.ctrl_interface.send_control(steering, self.target_speed)
            # log data
            data = (self.vehicle_model.x, self.vehicle_model.y,
                    self.vehicle_model.yaw, self.vehicle_model.v, self.time)
            self.data_log.append_data(*data)
            # update for animation
            if show_animation:
                self._animate_pure_pursuit(steering,cx,cy,target_ind)
            else:
                rospy.loginfo_throttle(1.5, self.vehicle_model)

        if show_animation:
            plt.close()
            self._animate_pure_pursuit(steering, cx, cy, target_ind)
            plt.show()
        else:
            # just show resulting plot if not animating
            self._plot_trajectory(cx, cy)
            plt.show()

    def drive_forward(self, show_animation = True):
        """Car follows straigth trajectory of predefined length.
            Uses line following algorithm to get to goal."""
        l = 1 # goal distance
        tol = 0.1 # ok distance to goal
        state = self.vehicle_model.get_state()
        x0 = self.target_state[0]
        y0 = self.target_state[1]
        xg = x0 + l*math.cos(state[2])
        yg = y0 + l*math.sin(state[2])
        at_goal = False
        while not at_goal and not rospy.is_shutdown():
            state = self.vehicle_model.get_state()
            x = state[0]
            y = state[1]
            yaw = state[2]
            # calculate and send control to car
            velocity, steering = lf.line_follower(x, y, yaw, x0, y0, xg, yg)
            self.ctrl_interface.send_control(steering,velocity)
            # log data

            data = tuple(self.vehicle_model.get_state())
            self.data_log.append_data(*data)
            if show_animation:
                self._animate_robot_path(steering, x0, y0, xg, yg)

            # done if close enough to goal.
            dist = np.linalg.norm([xg-x,yg-y])
            if dist < tol:
                at_goal = True
        self.target_state[0] = xg
        self.target_state[1] = yg
        print('Completed drive forward!')

    def turn_right(self):
        """Makes a full 90 degree right turn."""
        angle = self.target_state[2]-math.pi/2
        self.target_state[2] = angle
        self._turn(angle)
    def turn_left(self):
        """Makes a full 90 degree left turn."""
        angle = self.target_state[2]+math.pi/2
        self.target_state[2] = angle
        self._turn(angle)

def main():
    rospy.init_node('SVEA_sim_Environment')
    # Trajectory
    cx = np.arange(0, 5, 0.1)
    cy = [math.sin(ix) * ix for ix in cx]
    car = CarHighLevelCommands()
    from_file = True
    if from_file:
        # File with the code to execute
        dir_path = os.path.dirname(os.path.realpath(__file__))
        filename = dir_path + '/../../../../../../myapp/code.py'
        c = cc.CarCommands(filename)
        # variables to pass on
        g_var = {'CarHighLevelCommands': CarHighLevelCommands}
        l_var = {'car': car}
        c.execute_commands(g_var,l_var)
if __name__ == '__main__':
    main()
