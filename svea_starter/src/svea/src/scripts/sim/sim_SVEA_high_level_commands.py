#!/usr/bin/env python

# High level interface to simulate the SVEA cars.
# Written by Mikael Glamheden
# Last updated: 2019-07-17



import sys
import os
import rospy
import math
import numpy as np
import matplotlib.pyplot as plt
import time

import dependencies.line_follower as lf
import dependencies.car_commands as cc
import dependencies.data_log as dlog

import json

dirname = os.path.dirname(__file__)
svea = os.path.join(dirname, '../../')
sys.path.append(os.path.abspath(svea))

from models.bicycle_simple import SimpleBicycleState, plot_car
from simulators.sim_SVEA_simple import SimSVEA
from controllers.control_interfaces_sim import ControlInterface

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
                       animation = True,
                       vehicle_name = 'SVEA0',
                       init_state = [-1.35, -1.35, 0, 0],
                       goal = [0, 0]):
        self.simulation = simulation
        self.show_animation = animation
        self.vehicle_name = vehicle_name
        qualisys_model_name = vehicle_name
        dt = 0.01
        self.goal = goal
        if self.simulation:
            # initialize simulated model and control interface
            self.vehicle_model = SimpleBicycleState(*init_state, dt=dt)
            # self.ctrl_interface = ControlInterface(vehicle_name).start()
            # rospy.sleep(0.2)
            # start background simulation thread
            # self.simulator = SimSVEA(vehicle_name, self.vehicle_model, dt, is_publish=True)
            # self.simulator.start()
            self.target_state = init_state

            #rospy.sleep(0.5)
        else:
            # initialize odometry and control interface for real car
            self.vehicle_model = ql.QualisysSimpleOdom(qualisys_model_name).start()
            self.ctrl_interface = ControlInterface().start()
            rospy.sleep(1) # Wait till odometry updated

            self.target_state = self.vehicle_model.get_state() # x, y, yaw, v

        self.r = rospy.Rate(30) #S VEA car opearates on 30 Hz
        self.target_speed = 0.6 # [m/s]

        # object to log data in
        self.data_log = dlog.Data_log()

    def _send_position(self, steering):
        '''Send position to stdout as JSON data'''
        state = self.vehicle_model.get_state()
        data = {'x': state[0], 'y': state[1], 'yaw': state[2], 'v': state[3], 'steering': steering}
        sys.stdout.write(json.dumps(data))

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

    def _turn(self, angle, direction):
        """Turn the car a given number of degrees relative to current orientation."""
        l = 1
        tol1 = 1.1*math.pi/180 # angular tolerance
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
            # self.ctrl_interface.send_control(steering,velocity)
            self.vehicle_model.update(steering, velocity)
            # log data
            data = tuple(self.vehicle_model.get_state())
            self.data_log.append_data(*data)
            if self.show_animation:
                self._animate_robot_path(steering, x0, y0, xg, yg)

            #dist = np.linalg.norm([x0-x,y0-y])
            # Done if angle is close enough
            if abs(yaw_ref-yaw) < tol1:
                at_goal = True
            # send position to server.
            self._send_position(steering)
            # sleep so loop runs at 30Hz
            self.r.sleep()
        # self.ctrl_interface.send_control(0,0)
        self.vehicle_model.update(0, 0)
        self.target_state[0] = x
        self.target_state[1] = y
        print('[' + self.vehicle_name + '] : ' + 'Completed turn!')

    def at_goal(self):
        """Checks if car is close enough to goal."""
        state = self.vehicle_model.get_state()
        dist = np.linalg.norm(np.subtract(state[0:2],self.goal))
        if dist < 0.1:
            return True
        else:
            return False

    def drive_backwards(self):
        """Car follows straigth trajectory of predefined length.
            Uses line following algorithm to get to goal."""
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
            # self.ctrl_interface.send_control(steering, velocity) # Reverse steering.
            self.vehicle_model.update(steering, velocity)
            # log data
            data = tuple(self.vehicle_model.get_state())
            self.data_log.append_data(*data)
            if self.show_animation:
                self._animate_robot_path(steering, x0, y0, xg, yg)
            # done if close enough to goal
            dist = np.linalg.norm([xg-x,yg-y])
            if dist < tol:
                at_goal = True
            # Send position to server.
            self._send_position(steering)
            # sleep so loop runs at 30Hz
            self.r.sleep()
        self.target_state[0] = xg
        self.target_state[1] = yg
        # print('[' + self.vehicle_name + '] : ' + 'Completed drive backwards!')
        # print('State:'  + str(self.vehicle_model.get_state))

    def drive_forward(self):
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
            # self.ctrl_interface.send_control(steering, velocity)
            self.vehicle_model.update(steering, velocity)
            # log data
            data = tuple(self.vehicle_model.get_state())
            self.data_log.append_data(*data)
            if self.show_animation:
                self._animate_robot_path(steering, x0, y0, xg, yg)
            # done if close enough to goal
            dist = np.linalg.norm([xg-x,yg-y])
            if dist < tol:
                at_goal = True
            # Send position to server.
            self._send_position(steering)
            # sleep so loop runs at 30Hz
            self.r.sleep()
        self.target_state[0] = xg
        self.target_state[1] = yg
        # print('[' + self.vehicle_name + '] : ' + 'Completed drive forward!')
        # print('State:'  + str(self.vehicle_model.get_state))

    def turn_right(self):
        """Makes a full 90 degree right turn."""
        direction = 'R'
        angle_add = math.pi/2
        angle = lf.normalize_angle(self.target_state[2] - angle_add)
        self.target_state[2] = angle
        self._turn(angle, direction)
        # print('State:'  + str(self.vehicle_model.get_state))

    def turn_left(self):
        """Makes a full 90 degree left turn."""
        direction = 'L'
        angle_add = math.pi/2
        angle = lf.normalize_angle(self.target_state[2] + angle_add)
        self.target_state[2] = angle
        self._turn(angle, direction)
        # print('State:'  + str(self.vehicle_model.get_state))

def log_to_file(log):
    """Logs the cars path during execution to a file"""
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
    print('Wrote log to file')

def deploy(name, start, goal):
    """Wraps the init of car commands."""
    rospy.init_node('sim_SVEA_high_level_' + name)
    simulation = True
    animation = False
    car = CarHighLevelCommands(simulation, animation, name, start, goal)
    return car

def main(argv = ['SVEA0',
                 '{"x": 4, "y": 0, "yaw": 0}',
                 '{"x": 0, "y": 0, "yaw": 0}']):
    name = argv[0] # makes it possible to have multiple copies of simulation
    start = json.loads(argv[1])
    start = [start['x'], start['y'], start['yaw'], 0]
    goal = json.loads(argv[2])
    goal = [goal["x"], goal["y"]]

    # name = 'SVEA0'
    # goal = [4, 0]

    from_file = True
    if from_file:
        rover = deploy(name, start, goal) # Should be part of the code later on.
        # File with the code to execute
        dir_path = os.path.dirname(os.path.realpath(__file__))
        filename = dir_path + '/../../../../../../myapp/code.py'
        c = cc.CarCommands(filename)
        # variables to pass on
        g_var = {'CarHighLevelCommands': CarHighLevelCommands}
        l_var = {'rover': rover}
        c.execute_commands(name, g_var, l_var)
    else:
        # test code.
        car = deploy(name, goal)
        car.turn_right()
        car.turn_right()
        car.turn_right()
        car.drive_backwards()
        car.drive_backwards()
    log_to_file(rover.data_log)
    rospy.signal_shutdown('Program end')

if __name__ == '__main__':
    main(sys.argv[1:])
    # main()
