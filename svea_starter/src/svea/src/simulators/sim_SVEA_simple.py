#!/usr/bin/env python

"""
Simulation object for the SML SVEA platform.

Creates fake ROS subscriptions and publications that match the real car
platform. Intended for use to debug code BEFORE running on a real car.

Author - Frank Jiang (frankji@kth.se)
"""


import sys
import os
import math
from threading import Thread
import matplotlib.pyplot as plt

import rospy
from low_level_interface.msg import lli_ctrl_request, lli_ctrl_actuated

dirname = os.path.dirname(__file__)
svea = os.path.join(dirname, '../')
sys.path.append(os.path.abspath(svea))


class SimSVEA:
    """
    Simulation of a SVEA vehicle intended for debugging purposes,
    NOT for experiments.
    """

    OPERATING_FREQ = 30 # [Hz]

    MAX_VELOCITY = 1.0 # [m/s]

    def __init__(self, vehicle_name, initialized_model, dt, is_publish=False):
        """Initialization of simulation.

        Arguments:
            vehicle_name -- (string) name of vehicle, etc. SVEA0
            initialized_ -- (model obj) already initialized model
            dt           -- (float) sim time step
            is_publish   -- (bool) whether sim publishes to ROS or not
        """

        self.vehicle_name = vehicle_name

        model_dt = initialized_model.get_dt()
        if not model_dt == dt:
            rospy.loginfo("Mismatch between model's dt and "
                          +"simulation dt, Setting model's dt"
                          +"to simulation's")
            initialized_model.set_dt(dt)

        self.model = initialized_model
        self.dt = dt

        self.curr_ctrl_request = lli_ctrl_request()
        self.curr_ctrl_actuated = lli_ctrl_actuated()

        self.tic = rospy.get_time()

    def start(self):
        Thread(target=self._init_and_spin_ros, args=()).start()
        return self

    def _init_and_spin_ros(self):
        rospy.loginfo("Starting Simulation Node: \n"
                      + str(self))
        self.node_name = "simulated_" + self.vehicle_name
        self._collect_srvs()
        self._start_publish()
        self._start_listen()

    def _collect_srvs(self):
        pass

    def _start_listen(self):
        rospy.Subscriber(self.vehicle_name+'/lli/ctrl_request',
                         lli_ctrl_request, self._update_ctrl_request)
        rospy.loginfo("Simulation successfully initialized")

        self._start_simulation()

    def _start_publish(self):
        self.ctrl_actuated_pub = rospy.Publisher(self.vehicle_name+'/lli/ctrl_actuated',
                                                 lli_ctrl_actuated,
                                                 queue_size = 1)

    def _percent_to_steer(self, steer_percent):
        steer_percent = -float(steer_percent) # force float-based computations
        if steer_percent >= 0:
            steer_PWM = steer_percent / 100 * 600 + 1500
            sqrt_term = math.sqrt(5.016**2 - 4 * 0.2384 * (1495-steer_PWM))
            steering = (5.016 - sqrt_term) / (2*0.2384)
        else:
            steer_PWM = steer_percent / 100 * 600 + 1500
            sqrt_term = math.sqrt(5.016**2 - 4 *-0.2384 * (1495-steer_PWM))
            steering = (5.016 - sqrt_term) / (2*-0.2384)
        return math.radians(steering)

    def _percent_to_vel(self, vel_percent):
        vel_percent = float(vel_percent)
        velocity = vel_percent*self.MAX_VELOCITY / 100
        return velocity

    def _start_simulation(self):
        r = rospy.Rate(self.OPERATING_FREQ)

        while not rospy.is_shutdown():
            self.toc = rospy.get_time()

            steer_percent = self.curr_ctrl_request.steering
            vel_percent = self.curr_ctrl_request.velocity

            steering = self._percent_to_steer(steer_percent)
            velocity = self._percent_to_vel(vel_percent)

            self.model.update(steering, velocity)

            r.sleep() # force update frequency to be realistic

            # only update steering and velocity in simulation
            self.curr_ctrl_actuated.steering = steering
            self.curr_ctrl_actuated.velocity = velocity
            self.ctrl_actuated_pub.publish(self.curr_ctrl_actuated)

    def _update_ctrl_request(self, ctrl_request_msg):
        self.curr_ctrl_request = ctrl_request_msg
        self.tic = rospy.get_time()

    def _build_param_printout(self):
        param_str = "### {0} Simulation:\n".format(self.vehicle_name)

        param_str += "{0}\n".format(self.model)
        param_str += "### Sim dt: {0}\n".format(self.dt)

        param_str += "### Current Control Request: \n"
        # collect important params
        steering = self.curr_ctrl_request.steering
        velocity = self.curr_ctrl_request.velocity
        transmission = self.curr_ctrl_request.transmission
        differential_front = self.curr_ctrl_request.differential_front
        differential_rear = self.curr_ctrl_request.differential_rear
        ctrl_code = self.curr_ctrl_request.ctrl_code

        param_str += ("## Vehicle: {0}\n".format(self.vehicle_name)
                      +"  -ctrl request:\n"
                      +"      steering   - {0}\n".format(steering)
                      +"      velocity   - {0}\n".format(velocity)
                      +"      trans      - {0}\n".format(transmission)
                      +"      diff_front - {0}\n".format(differential_front)
                      +"      diff_rear  - {0}\n".format(differential_rear)
                      +"      ctrl_code  - {0}\n".format(ctrl_code))

        return param_str

    def __repr__(self):
        return self._build_param_printout()
    def __str__(self):
        return self._build_param_printout()

def main():

    rospy.init_node('SVEA_simulator')

    vehicle_name = rospy.get_param('/SVEA_simulator/vehicle_name', "SVEA")
    model_name = rospy.get_param('/SVEA_simulator/model_name', "SimpleBicycle")

    # load in model from SVEA model library
    if model_name == "SimpleBicycle":
        from models.bicycle_simple import SimpleBicycleState
        Model = SimpleBicycleState
    else:
        print("! ### Given model name ({0}) not handled.\n".format(model_name)
              + "Terminating Simulation.")
        return

    default_init = [0 for _ in range(Model().get_state_dim())]
    init_state = rospy.get_param('/SVEA_simulator/init_state', default_init)
    dt = rospy.get_param('/SVEA_simulator/dt', 0.1)

    SVEA_model = Model(*init_state, dt=dt)
    SimSVEA(vehicle_name, SVEA_model, dt, is_publish=True)
    SimSVEA.start()


if __name__ == '__main__':
    main()
