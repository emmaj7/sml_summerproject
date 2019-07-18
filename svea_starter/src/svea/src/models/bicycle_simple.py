#!/usr/bin/env python

"""
Simple bicycle model with params set for SVEA cars.

State-space eqn:
    \dot{x}     = v * cos(\phi)
    \dot{y}     = v * sin(\phi)
    \dot{\phi}  = v/L * tan(\delta)
    \dot{v}     = a

    state: (x, y, \phi, v) -> (x pos, y pos, yaw, velocity)
    input: (\delta, a) -> (steering, acceleration)
    param: (L, \delta_{max}) -> (wheel base, max steering)

    units: [m, rad]

author: Frank J Jiang (frankji@kth.se)

"""


import numpy as np
import math
import matplotlib.pyplot as plt


class SimpleBicycleState(object):
    """ Adopted from PythonRobotics for compatibility. """

    L = 0.32
    DELTA_MAX = np.radians(30.0)  # max steering

    TAU = 0.1 # gain for simulating SVEA's ESC

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, dt=0.1):
        """ Initialize state. """

        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

        self.dt = dt

    def _normalize_angle(self, angle):
        """ Normalize an angle to [-pi, pi]. """
        while angle > np.pi:
            angle -= 2.0 * np.pi
        while angle < -np.pi:
            angle += 2.0 * np.pi

        return angle

    def _build_param_printout(self):
        return ("## Simple Bicycle State:\n"
                + "  - x: {0}".format(self.x)
                + "  - y: {0}".format(self.y)
                + "  - yaw: {0}".format(self.yaw)
                + "  - v: {0}".format(self.v)
                + "  - dt: {0}".format(self.dt))

    def __repr__(self):
        return self._build_param_printout()
    def __str__(self):
        return self._build_param_printout()

    def _sim_esc(self, target_velocity):
        """ Simulates the affect of the ESC on SVEA's chassis.

        Arguments
        """
        return 1/self.TAU * (target_velocity - self.v)

    def _update(self, accel, delta):
        """ Update state using simple bicycle model dynamics """

        delta = np.clip(delta, -self.DELTA_MAX, self.DELTA_MAX)

        self.x += self.v * np.cos(self.yaw) * self.dt
        self.y += self.v * np.sin(self.yaw) * self.dt
        self.yaw += self.v / self.L * np.tan(delta) * self.dt
        self.yaw = self._normalize_angle(self.yaw)
        self.v += accel * self.dt

    def update(self, steering, velocity,
                     transmission = 0,
                     differential_front = 0,
                     differential_rear = 0,
                     ctrl_code = 0):
        """
        Use a wrapper for updates to provide standard update function
        that supports lli control requests.
        """

        accel = self._sim_esc(velocity)
        delta = steering

        self._update(accel, delta)

    def get_state(self):
        state = [self.x,
                 self.y,
                 self.yaw,
                 self.v]
        return np.asarray(state)

    def get_readable_state(self):
        state = {"x": self.x,
                 "y": self.y,
                 "yaw": self.yaw,
                 "v": self.v}
        return state

    def get_state_dim(self):
        return len(self.get_state())

    def correct_state(self, x, y, yaw):
        self.x = x
        self.y = y
        self.yaw = yaw

    def set_dt(self, dt):
        self.dt = dt

    def get_dt(self):
        return self.dt


## Plotting function from PythonRobotics MPC ##################################
# TODO: integrate into state object

# Vehicle parameters
LENGTH = 0.586  # [m]
WIDTH = 0.2485  # [m]
BACKTOWHEEL = 0.16  # [m]
WHEEL_LEN = 0.03  # [m]
WHEEL_WIDTH = 0.02  # [m]
TREAD = 0.07  # [m]
WB = 0.324  # [m]

def plot_car(x, y, yaw, steer=0.0, color="-k"):

    outline = np.matrix([[-BACKTOWHEEL, (LENGTH - BACKTOWHEEL), (LENGTH - BACKTOWHEEL), -BACKTOWHEEL, -BACKTOWHEEL],
                         [WIDTH / 2, WIDTH / 2, - WIDTH / 2, -WIDTH / 2, WIDTH / 2]])

    fr_wheel = np.matrix([[WHEEL_LEN, -WHEEL_LEN, -WHEEL_LEN, WHEEL_LEN, WHEEL_LEN],
                          [-WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD, WHEEL_WIDTH - TREAD, WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD]])

    rr_wheel = np.copy(fr_wheel)

    fl_wheel = np.copy(fr_wheel)
    fl_wheel[1, :] *= -1
    rl_wheel = np.copy(rr_wheel)
    rl_wheel[1, :] *= -1

    Rot1 = np.matrix([[math.cos(yaw), math.sin(yaw)],
                      [-math.sin(yaw), math.cos(yaw)]])
    Rot2 = np.matrix([[math.cos(steer), math.sin(steer)],
                      [-math.sin(steer), math.cos(steer)]])

    fr_wheel = (fr_wheel.T * Rot2).T
    fl_wheel = (fl_wheel.T * Rot2).T
    fr_wheel[0, :] += WB
    fl_wheel[0, :] += WB

    fr_wheel = (fr_wheel.T * Rot1).T
    fl_wheel = (fl_wheel.T * Rot1).T

    outline = (outline.T * Rot1).T
    rr_wheel = (rr_wheel.T * Rot1).T
    rl_wheel = (rl_wheel.T * Rot1).T

    outline[0, :] += x
    outline[1, :] += y
    fr_wheel[0, :] += x
    fr_wheel[1, :] += y
    rr_wheel[0, :] += x
    rr_wheel[1, :] += y
    fl_wheel[0, :] += x
    fl_wheel[1, :] += y
    rl_wheel[0, :] += x
    rl_wheel[1, :] += y

    plt.plot(np.array(outline[0, :]).flatten(),
             np.array(outline[1, :]).flatten(), color)
    plt.plot(np.array(fr_wheel[0, :]).flatten(),
             np.array(fr_wheel[1, :]).flatten(), color)
    plt.plot(np.array(rr_wheel[0, :]).flatten(),
             np.array(rr_wheel[1, :]).flatten(), color)
    plt.plot(np.array(fl_wheel[0, :]).flatten(),
             np.array(fl_wheel[1, :]).flatten(), color)
    plt.plot(np.array(rl_wheel[0, :]).flatten(),
             np.array(rl_wheel[1, :]).flatten(), color)
    plt.plot(x, y, "*")

###############################################################################
