from distutils.command.config import config
from typing import Optional, Union, Tuple, Dict, Any, List

import numpy as np
import cv2
import yaml
import os
import octomap
import map_wrapper
import time

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
        self.frame_time = 1000/self.fps #ms
        self.action_mode = config['drone']['action_mode']
        self.vel_ratio = config['drone']['vel_linearly_ratio']
        self.vel_yaw_ratio = config['drone']['vel_yaw_ratio']
        self.camera_pram = config['drone']['camera_pram']
        self.global_map_size = config['map']['global_map_size']
        self.loacl_map_size = config['map']['loacl_map_size']
        self.resolution = config['map']['resolution']

        # STEP 2 Init  MultirotorClient
        self._drone_client = airsim.MultirotorClient(ip = "127.0.0.1", port = self.client_port)
        self._drone_client.confirmConnection()
        self._drone_client.enableApiControl(True, drone_name)
        self._drone_client.armDisarm(True, drone_name)

        # STEP 3 Init Map
        # local/global_last/global_last
        self.octotree = octomap.OcTree(self.resolution)
        self.global_map = np.ones(self.loacl_map_size,dtype=np.uint8)
        self.last_global_map = np.ones(self.loacl_map_size,dtype=np.uint8)
        self.local_map = np.ones(self.loacl_map_size,dtype=np.uint8)
        self.last_time_stamp = np.uint64(0)
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

    def _get_collision_state(self):
        return self._drone_client.simGetCollisionInfo(vehicle_name = self.drone_name)

    def _get_depth_img(self):
        return self._drone_client.simGetImages([airsim.ImageRequest("front_center_custom", airsim.ImageType.DepthPlanar,
                                                    True, False)],vehicle_name = self.drone_name)[0]    

    def _update_map(self):
        """update global octree use current frame of depth

        Returns:
            np.uint64: the time_stamp of the instert depth img
        """
        img_response = self._get_depth_img()
        r,p,y = map_wrapper.quaternion_to_euler(img_response.camera_orientation)
        frame_origin_np = np.array(img_response.camera_position.x_val,img_response.camera_position.y_val,
                                    img_response.camera_position.z_val,r,p,y)
        depth_img = airsim.list_to_2d_float_array(img_response.image_data_float, img_response.width, img_response.height)
        pcd = map_wrapper.pointcloud_from_depth(depth_img, fx=self.camera_pram['fx'], 
                                    fy=self.camera_pram['fy'], cx=self.camera_pram['cx'], cy=self.camera_pram['cy'])
        nonnan = ~np.isnan(pcd).any(axis=2)   
        self.octree.insertPointCloud(pointcloud=pcd[nonnan],sensor_origin=np.array([0, 0, 0],
                                    frame_origin = frame_origin_np, dtype=float), maxrange=6)
        return img_response.time_stamp

    def _rescure(self):
        pass

    def action_to_control(self, action_index):
        cur_action = self._action_set[action_index]
        if self.action_mode == 'velocity':
            cur_action = cur_action*self.vel_ratio
            cur_action[3] = cur_action[3]/self.vel_ratio*self.vel_yaw_ratio
            self._drone_client.moveByVelocityAsync(cur_action[0],cur_action[1],cur_action[2],self.frame_time,
                                                    yaw_mode = airsim.YawMode(is_rate = True,yaw_or_rate = cur_action[3]))
    
    def step_synchronous(self, action_ind: int):
        self.action_to_control(action_ind)
        time.sleep(self.frame_time/1000)
        self._update_map()
        pass

    def step_asynchronous(self, action_ind: int):
        pass

    def seed(self, seed: Optional[int] = None):
        pass

    def step(self, action_ind: int):
        #self._drone_client.simContinueForTime(0.05) #? or just run the drone for a few seconds
        # TODO Test how fast can get a img
        pass
        # #moveByAngleThrottleAsync
        response = self._get_depth_img()
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

        