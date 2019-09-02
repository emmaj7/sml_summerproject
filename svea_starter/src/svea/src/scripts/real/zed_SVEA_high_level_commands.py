#!/usr/bin/env python

# High level interface to run the SVEA cars. zed version.
# This version of the program uses zed camera for localization of the vehicle
# Written by Mikael Glamheden
# Last updated: 2019-07-17

import sys
import os
import rospy
import math
import numpy as np
import matplotlib.pyplot as plt
import json
import string
import demjson

import mikaels_code.line_follower as lf
import mikaels_code.car_commands as cc
import mikaels_code.data_log as dlog

from mikaels_code.odom_publisher import OdomPublisher
from mikaels_code.amcl_pose_subscriber import AmclPoseSubscriber
from mikaels_code.zed_pose_subscriber import PoseSubscriber

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
    def __init__(self, init_state = [0, 0, 0, 0]):
        dt = 1.0/30
        # initialize odometry and control interface for real car
        # self.vehicle_model = ql.QualisysSimpleOdom(qualisys_model_name).start()
        self.vehicle_model = SimpleBicycleState(*init_state, dt=dt)
        # self.odom_publisher = OdomPublisher().start()
        self.pose_node = PoseSubscriber().start()
        # state = self.vehicle_model.get_state()
        rospy.sleep(1)
        # self.odom_publisher.send_odometry(state, 0)

        self.ctrl_interface = ControlInterface().start()
        rospy.sleep(6) # Wait till odometry updated

        self.target_state = self.pose_node.get_state() # x, y, yaw, v

        self.r = rospy.Rate(30) #SVEA car opearates on 30 Hz
        # object to log data in
        self.data_log = dlog.Data_log()

    def _send_position(self, steering):
        '''Send position to stdout in JSON format.'''
        state = self.vehicle_model.get_state()
        data = {'x': state[0], 'y': state[1], 'yaw': state[2], 'v': state[3], 'steering': steering}
        # sys.stdout.write(json.dumps(data))

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
            # state = self.vehicle_model.get_state()
            state = self.pose_node.get_state()
            x = state[0]
            y = state[1]
            yaw = state[2]
            time = rospy.Time.now()
            # print('Rostime : %i %i', time.secs, time.nsecs)
            # print('Lst zed data : %d', state[3])
            # print('Time difference : %d', time-state[3])

            # calculate and send control to car
            velocity, steering = lf.orientation_controller(x, y, yaw, yaw_ref, direction)
            self.ctrl_interface.send_control(steering,velocity)

            # update model
            # self.vehicle_model.update(steering, velocity)

            # publish dead reckoning
            # self.odom_publisher.send_odometry(state, steering)

            # update with information from localization
            # self.pose_update()

            # log data
            data = tuple(self.vehicle_model.get_state())
            self.data_log.append_data(*data)

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

    def drive_backwards(self):
        """Car follows straigth trajectory of predefined length.
            Uses line following algorithm to get to goal."""
        # Go into reverse mode.
        self.ctrl_interface.send_control(0,-1)
        self.r.sleep()
        self.ctrl_interface.send_control(0,0)
        self.r.sleep()

        l = 0.5 # goal distance
        tol = 0.15 # ok distance to goal
        state = self.pose_node.get_state()
        x0 = self.target_state[0]
        y0 = self.target_state[1]

        # Reverse goal direction:
        xg = x0 - l*math.cos(self.target_state[2])
        yg = y0 - l*math.sin(self.target_state[2])

        at_goal = False
        while not at_goal and not rospy.is_shutdown():
            state = self.pose_node.get_state()
            x = state[0]
            y = state[1]
            yaw = state[2]
            # calculate and send control to car
            velocity, steering = lf.line_follower_reverse(x, y, yaw, x0, y0, xg, yg)
            self.ctrl_interface.send_control(steering, velocity) # Reverse steering.

            # log data
            data = tuple(self.pose_node.get_state())
            self.data_log.append_data(*data)
            # done if close enough to goal
            dist = np.linalg.norm([xg-x,yg-y])
            if dist < tol:
                at_goal = True
                self.ctrl_interface.send_control(0,0)
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
        state = self.pose_node.get_state()
        x0 = self.target_state[0]
        y0 = self.target_state[1]
        xg = x0 + l*math.cos(self.target_state[2])
        yg = y0 + l*math.sin(self.target_state[2])
        at_goal = False
        while not at_goal and not rospy.is_shutdown():
            state = self.pose_node.get_state()
            x = state[0]
            y = state[1]
            yaw = state[2]
            # calculate and send control to car
            velocity, steering = lf.line_follower(x, y, yaw, x0, y0, xg, yg)
            self.ctrl_interface.send_control(steering, velocity)

            # log data
            data = tuple(self.pose_node.get_state())
            self.data_log.append_data(*data)
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
    car = CarHighLevelCommands()
    return car

def main(argv = ['code_real.py']):
    from_file = True
    if from_file:

        dir_path = os.path.dirname(os.path.realpath(__file__))
        filename = dir_path + '/../../../../../../myapp/' + argv[0]

        c = cc.CarCommands(filename)
        name, length = c.get_shortest_code()

        print('---------------------------------------------')
        print('Running code with id ' + name + ' which is %d lines long' % length)
        print('---------------------------------------------')

        rover = deploy(name) # This should be part of the code later on.

        # variables to pass on
        g_var = {'CarHighLevelCommands': CarHighLevelCommands}
        l_var = {'rover': rover}
        c.execute_commands(name, g_var, l_var)
    else:
        name = 'SVEA5'
        rover = deploy(name)
        # rover.drive_forward()
        # rover.turn_left()
        # rover.turn_left()
        # rover.turn_right()
        # rover.turn_right()
        # rover.drive_forward()
        # rover.drive_forward()
        # rover.drive_forward()
        # rover.turn_right()
        # rover.drive_forward()
        # rover.drive_forward()
        # rover.drive_forward()
        rover.turn_left()
        rover.turn_right()
        rover.turn_left()
        rover.turn_right()
        rover.drive_backwards()
        rover.turn_right()
        rover.drive_forward()
        rover.drive_forward()
        rover.drive_forward()

        # car.drive_forward()
    log_to_file(rover.data_log)
    rospy.signal_shutdown('Program end')
if __name__ == '__main__':
    main(sys.argv[1:])
    # main()
