#!/usr/bin/env python
import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from airsim_ros_pkgs.msg import VelCmd
from airsim_ros_pkgs.srv import TakeoffRequest,Takeoff

import numpy as np

class Drone:
    def __init__(self, drone_name):
        # TODO Add MutilMaps  
        # Step 1 init ros handeler
        # Step 2 init ros sub and pub
        # sub(or get message) poisition, grid_map, imu_data, collision_check
        # pub action, reset_signal
        # Step 3 init action space
        # Step 4 Read some parma
        pass

    def step(self, action):
        pass

    def rescuer(self):
        pass

    def reset_env(self):
        pass
    

