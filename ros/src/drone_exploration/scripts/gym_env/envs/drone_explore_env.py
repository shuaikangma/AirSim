from distutils.command.config import config
from typing import Optional, Union, Tuple, Dict, Any, List

import numpy as np
import cv2
import yaml
import os

import gym
import gym.logger as logger
from gym import error, spaces
from gym import utils

import airsim

class DroneExploreEnv(gym.Env):
#class DroneExploreEnv():
    def __init__(
        self, 
        drone_name: str = 'Drone_1',
        client_port: float = 41451,
        config_path: str = '../configs/env_config.yaml'
        ) -> None:
        super().__init__()
        #ActionSpace Mapping: https://www.zhihu.com/question/37189447/answer/74759345
        # STEP 1 Init Config
        self.drone_name = drone_name
        self.client_port = client_port
        yaml_config = open(config_path,'r',encoding='utf-8').read()
        config = yaml.safe_load(yaml_config)
        self.time_ratio = config['env']['time_ratio']
        self.fps = config['env']['fps']
        self.action_mode = config['drone']['action_mode']
        self.camera_pram = config['drone']['camera_pram']
        self.loacl_map_size = config['map']['loacl_map_size']

        # STEP 2 Init  MultirotorClient
        self._drone_client = airsim.MultirotorClient(ip = "127.0.0.1", port = self.client_port)
        self._drone_client.confirmConnection()
        self._drone_client.enableApiControl(True, drone_name)
        self._drone_client.armDisarm(True, drone_name)

        # STEP 3 Init Map
        # local/global_last/global_last
        self._init_map()

        # STEP 4 Gym Var
        self._action_set = np.genfromtxt('../configs/action_space.csv', delimiter=",")
        self._action_space = spaces.Discrete(len(self._action_set))
        #obs_space: 0-free, 1-unknow, 2-previous_path, 3-occupy
        self._obs_space = spaces.Box(
                low=0, high=4, dtype=np.uint8, shape=self.loacl_map_size
            )

    def _init_map(self):
        # step0 construct self.octotree 
        # step1 get depth img
        # step2 pointcloud_fprint(type(img2d))rocess every xxx ms
        # step5 occupied, empty = octree.extractPointCloud()
        # step6 add dron_position and convert to numpy/box
        pass
        self.last_global_map = self.global_map = None
        self.local_map = None

    def _update_map(self, depth_img):
        pass

    def _get_collision_state(self):
        pass

    def _rescure(self):
        pass

    def action_to_control(self, action_index):
        pass
    
    def seed(self, seed: Optional[int] = None):
        pass

    def step(self, action_ind: int):
        #self._drone_client.simContinueForTime(0.05) #? or just run the drone for a few seconds
        # TODO Test how fast can get a img
        pass
        # #moveByAngleThrottleAsync
        #self._drone_client.simGetImage(camera_name = "front_center_custom", image_type = airsim.ImageType.Scene, vehicle_name = self.drone_name)
        responses = self._drone_client.simGetImages([airsim.ImageRequest("front_center_custom", airsim.ImageType.DepthPlanar,
                                                    True, False)],vehicle_name = self.drone_name)
        response = responses[0]
        # get numpy array
        img2d = airsim.list_to_2d_float_array(response.image_data_float, response.width, response.height) 
        print(type(img2d))
        cv2.imshow(self.drone_name,img2d)
        cv2.waitKey(2)
        print('camera_position',response.camera_position)
        print('camera_orientation',response.camera_orientation)

        # reshape array to 4 channel image array H X W X 4
        #img_rgb = img1d.reshape(response.height, response.width, 1)
        print(response.time_stamp)
        return response.time_stamp

    def reset(self):
        self._drone_client.reset() # or just set position to orgion
        pass

    def close(self) -> None:
        """
        Cleanup any leftovers by the environment
        """
        pass

    def render(self, mode: str):
        pass

    def manual_control(self):
        #start joystick control thread
        pass

        