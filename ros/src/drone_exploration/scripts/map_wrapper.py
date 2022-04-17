#!/usr/bin/env python
import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from airsim_ros_pkgs.msg import VelCmd
from airsim_ros_pkgs.srv import TakeoffRequest,Takeoff

import numpy as np

class MapWrapper:
    def __init__(self, drone_name):
        # TODO octomap sub
        # get mesh map(https://github.com/microsoft/AirSim/pull/3209)
        # update global map
        # update local map(need position)
        # 
        pass