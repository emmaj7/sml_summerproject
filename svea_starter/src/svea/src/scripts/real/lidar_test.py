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

from sensors.lidars import RPLidar

def main():
    rospy.init_node('lidar-test')
    r = rospy.Rate(30)
    lidar = RPLidar()
    rospy.sleep(2)
    lidar.start()
    lidar.start_motor()
    while not rospy.is_shutdown():
        print(RPlidar.get_raw_scan())
        r.sleep()

if __name__ == '__main__':
    main()
