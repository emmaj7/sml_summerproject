#!/usr/bin/env python

# High level interface to run the SVEA cars.
# Written by Mikael Glamheden
# 2019-06-04

import sys
import os
import rospy
import math
import numpy as np
import matplotlib.pyplot as plt
import json

import demjson

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
    def __init__(self, simulation = True,
                       init_state = [0, 0, 0, 0]):
        self.simulation = simulation
        # self.show_animation = animation
        qualisys_model_name = 'SVEA5'
        dt = 0.01
        if self.simulation:
            # initialize simulated model and control interface
            self.vehicle_model = SimpleBicycleState(*init_state, dt=dt)
            self.ctrl_interface = ControlInterface(qualisys_model_name).start()
            rospy.sleep(0.5)
            # start background simulation thread
            self.simulator = SimSVEA(vehicle_name, self.vehicle_model, dt, is_publish=True)
            self.simulator.start()
            self.target_state = [0, 0, 0, 0]
            rospy.sleep(0.5)
        else:
            # initialize odometry and control interface for real car
            self.vehicle_model = ql.QualisysSimpleOdom(qualisys_model_name).start()
            self.ctrl_interface = ControlInterface().start()
            rospy.sleep(2) # Wait till odometry updated

            self.target_state = self.vehicle_model.get_state() # x, y, yaw, v

        self.r = rospy.Rate(30) #S VEA car opearates on 30 Hz
        self.target_speed = 0.6 # [m/s]
        # object to log data in
        self.data_log = dlog.Data_log()

    def _send_position(self, steering):
        # Send position to server using JSON
        state = self.vehicle_model.get_state()
        data = {'x': state[0], 'y': state[1], 'yaw': state[2], 'v': state[3], 'steering': steering}
        # sys.stdout.write(json.dumps(data))

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

    def _animate_robot_path(self, steering, x0, y0, xg, yg):
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

    def _turn(self, angle, direction):
        """Turn the car a given number of degrees relative to current orientation."""
        l = 1
        tol1 = 5*math.pi/180 # angular tolerance
        tol2 = 2 # positional tolerance
        x0 = self.target_state[0]
        y0 = self.target_state[1]
        xg = x0 + l*math.cos(angle)
        yg = y0 + l*math.sin(angle)

        yaw_ref = angle
        at_goal = False
        while not at_goal and not rospy.is_shutdown():
            state = self.vehicle_model.get_state()
            x = state[0]
            y = state[1]
            yaw = state[2]
            # calculate and send control to car
            velocity, steering = lf.orientation_controller(x, y, yaw, yaw_ref, direction)
            self.ctrl_interface.send_control(steering,velocity)

            # log data
            data = tuple(self.vehicle_model.get_state())
            self.data_log.append_data(*data)
            if self.simulation:
                self._animate_robot_path(steering, x0, y0, xg, yg)

            #dist = np.linalg.norm([x0-x,y0-y])
            # Done if angle is close enough
            if abs(yaw_ref-yaw) < tol1:
                at_goal = True
                self.ctrl_interface.send_control(0,0)

            self._send_position(steering)
            # sleep so loop runs at 30Hz
            self.r.sleep()

        self.ctrl_interface.send_control(0,0)
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
            if self.simulation:
                self._animate_pure_pursuit(steering,cx,cy,target_ind)
            else:
                rospy.loginfo_throttle(1.5, self.vehicle_model)
            # sleep so loop runs at 30Hz
            self.r.sleep()

        if self.simulation:
            plt.close()
            self._animate_pure_pursuit(steering, cx, cy, target_ind)
            plt.show()
        else:
            # just show resulting plot if not animating
            self._plot_trajectory(cx, cy)
            plt.show()

    def drive_backwards(self):
        """Car follows straigth trajectory of predefined length.
            Uses line following algorithm to get to goal."""
        # Go into reverse mode.
        self.ctrl_interface.send_control(0,-1)
        self.r.sleep()
        self.ctrl_interface.send_control(0,0)
        self.r.sleep()

        l = 0.5 # goal distance
        tol = 0.1 # ok distance to goal
        state = self.vehicle_model.get_state()
        x0 = self.target_state[0]
        y0 = self.target_state[1]

        # Reverse goal direction:
        xg = x0 - l*math.cos(self.target_state[2])
        yg = y0 - l*math.sin(self.target_state[2])

        at_goal = False
        while not at_goal and not rospy.is_shutdown():
            state = self.vehicle_model.get_state()
            x = state[0]
            y = state[1]
            yaw = state[2]
            # calculate and send control to car
            velocity, steering = lf.line_follower_reverse(x, y, yaw, x0, y0, xg, yg)
            self.ctrl_interface.send_control(steering, velocity) # Reverse steering.

            # log data
            data = tuple(self.vehicle_model.get_state())
            self.data_log.append_data(*data)
            if self.simulation:
                self._animate_robot_path(steering, x0, y0, xg, yg)
            # done if close enough to goal
            dist = np.linalg.norm([xg-x,yg-y])
            if dist < tol:
                self.ctrl_interface.send_control(0,0)
                at_goal = True
            # Send position to server.
            self._send_position(steering)
            # sleep so loop runs at 30Hz
            self.r.sleep()
        self.ctrl_interface.send_control(0,0)
        self.target_state[0] = xg
        self.target_state[1] = yg
        print('Completed drive backwards!')

    def drive_forward(self, show_animation = True):
        """Car follows straigth trajectory of predefined length.
            Uses line following algorithm to get to goal."""
        l = 0.5 # goal distance
        tol = 0.1 # ok distance to goal
        state = self.vehicle_model.get_state()
        x0 = self.target_state[0]
        y0 = self.target_state[1]
        xg = x0 + l*math.cos(self.target_state[2])
        yg = y0 + l*math.sin(self.target_state[2])
        at_goal = False
        while not at_goal and not rospy.is_shutdown():
            state = self.vehicle_model.get_state()
            x = state[0]
            y = state[1]
            yaw = state[2]
            # calculate and send control to car
            velocity, steering = lf.line_follower(x, y, yaw, x0, y0, xg, yg)
            self.ctrl_interface.send_control(steering, velocity)
            # log data

            data = tuple(self.vehicle_model.get_state())
            self.data_log.append_data(*data)
            if self.simulation:
                self._animate_robot_path(steering, x0, y0, xg, yg)
            else:
                rospy.loginfo_throttle(1.5, self.vehicle_model)
            # done if close enough to goal
            dist = np.linalg.norm([xg-x,yg-y])
            #print('Current pos:')
            #print(state)
            if dist < tol:
                at_goal = True

            self._send_position(steering)
            # sleep so loop runs at 30Hz
            self.r.sleep()
        self.ctrl_interface.send_control(0,0)
        self.target_state[0] = xg
        self.target_state[1] = yg
        print('Completed drive forward!')

    def turn_right(self):
        """Makes a full 90 degree right turn."""
        direction = 'R'
        angle_add = math.pi/2
        angle = lf.normalize_angle(self.target_state[2] - angle_add)
        self.target_state[2] = angle
        self._turn(angle, direction)
        self.r.sleep()

    def turn_left(self):
        """Makes a full 90 degree left turn."""
        direction = 'L'
        angle_add = math.pi/2
        angle = lf.normalize_angle(self.target_state[2] + angle_add)
        self.target_state[2] = angle
        self._turn(angle, direction)
        self.r.sleep()

def log_to_file(log):
    print('Starting writing log')
    dir_path = os.path.dirname(os.path.realpath(__file__))
    f = open(dir_path+"/log_file.txt","w+")
    for e in log.get_x():
        f.write(str(e) + '\n')
    f.write('#\n')
    for e in log.get_y():
        f.write(str(e) + '\n')
    f.write('# \n')
    for e in log.get_yaw():
        f.write(str(e) + '\n')
    f.write('# \n')
    for e in log.get_v():
        f.write(str(e) + '\n')
    f.write('# \n')
    f.close()
    print('Wrote log')

def deploy(name):
    """Wraps the init of car commands."""
    rospy.init_node('SVEA_high_level_' + name)
    simulation = False
    car = CarHighLevelCommands(simulation)
    return car

def main(argv = ['SVEA5']):

    # print('argv:')
    # print(argv)

    name = argv[0] # determines which code snippet to take from file.
    # goal = demjson.decode(argv[1])
    # goal = [goal["x"], goal["y"]
    # name = 'SVEA5'

    from_file = False
    rover = deploy(name) # This should be part of the code later on.
    # car = CarHighLevelCommands(simulation)
    if from_file:
        dir_path = os.path.dirname(os.path.realpath(__file__))
        filename = dir_path + '/../../../../../../myapp/' + argv[1]
        print('filename %s' % filename)
        # File with the code to execute
        # filename = dir_path + '/../../../../../../myapp/code_real.py'
        c = cc.CarCommands(filename)

        # variables to pass on
        g_var = {'CarHighLevelCommands': CarHighLevelCommands}
        l_var = {'rover': rover}
        c.execute_commands(name, g_var, l_var)
    else:
        rover.drive_forward()
        rover.drive_backwards()
        # rover.drive_forward()
        # rover.drive_forward()
        # rover.drive_forward()
        # car.turn_right()
        # car.drive_forward()
    log_to_file(rover.data_log)
    # rospy.signal_shutdown('Program end')
if __name__ == '__main__':
    # main(sys.argv[1:])
    main()
