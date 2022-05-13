from distutils.command.config import config
from typing import Optional, Union, Tuple, Dict, Any, List

import numpy as np
import yaml
#import octomap
#from PythonClient.airsim.types import CollisionInfo
from map_wrapper import GridMap
from matplotlib import pyplot as plt
import time
import random

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
        config_path: str = '/home/mrmmm/DRL_Exploration_With_Airsim/ros/src/drone_exploration/scripts/gym_env/configs/env_config.yaml'
        ) -> None:
        super().__init__()
        #ActionSpace Mapping: https://www.zhihu.com/question/37189447/answer/74759345
        # STEP 1 Init Config
        self.drone_name = drone_name
        self.client_port = client_port
        yaml_instream = open(config_path,'r',encoding='utf-8')
        yaml_config = yaml_instream.read()
        config = yaml.safe_load(yaml_config)
        self.time_ratio = config['env']['time_ratio']
        self.fps = config['env']['fps']
        self.frame_time = 10**9*1.0/self.fps #ns
        self.step_mode = config['env']['step_mode']
        self.action_mode = config['drone']['action_mode']
        self.vel_ratio = config['drone']['vel_linearly_ratio']
        self.vel_yaw_ratio = config['drone']['vel_yaw_ratio']
        self.camera_pram = config['drone']['camera_pram']
        self.global_map_range = config['map']['global_map_range']
        self.loacl_map_range = config['map']['loacl_map_range']
        self.resolution = config['map']['resolution']
        yaml_instream.close()
        self.loacl_map_shape = [int(f/self.resolution) for f in self.loacl_map_range]

        # STEP 2 Init  MultirotorClient
        self._drone_client = airsim.MultirotorClient(ip = "127.0.0.1", port = self.client_port)
        self._drone_client.confirmConnection()
        self._drone_client.enableApiControl(True, drone_name)
        self._drone_client.armDisarm(True, drone_name)
        self._drone_client.takeoffAsync(vehicle_name = drone_name)
        print(self._get_collision_state())


        # STEP 3 Init Map
        # local/global_last/global_last
        self.grid_map = GridMap(config_path)
        #self.last_global_map = self.grid_map.global_map.copy()
        self.last_known_region = 0
        self.last_time_stamp = np.uint64(0)
        self._init_map()

        # STEP 4 Gym Var
        self._action_set = np.genfromtxt('../configs/action_space.csv', delimiter=",")
        self._action_space = spaces.Discrete(len(self._action_set))
        #obs_space: 0-free, 1-unknow, 2-previous_path, 3-occupy
        self._obs_space = spaces.Box(
                low=0, high=3, dtype=np.uint8, shape=self.grid_map.loacl_map_shape
            )

    def _init_map(self):
        self._drone_client.rotateByYawRateAsync(yaw_rate = 360, duration = 1.0, vehicle_name = self.drone_name)
        self.last_time_stamp = self._get_collision_state().time_stamp
        time.sleep(1)
        for _ in range(int(self.fps)):
            depth,position,cur_time_stamp = self._get_depth_img()
            self.grid_map.update_map(depth,position)
            if(cur_time_stamp-self.last_time_stamp < self.frame_time):
                time.sleep((self.frame_time -(cur_time_stamp-self.last_time_stamp))/10**9)
            else:
                print("超时！！！ 处理时间： ", (cur_time_stamp-self.last_time_stamp)/10**6, 'ms')
            self.last_time_stamp = cur_time_stamp
        self.last_global_map = self.grid_map.global_map  

    def _get_collision_state(self):
        return self._drone_client.simGetCollisionInfo(vehicle_name = self.drone_name)

    def _get_depth_img(self):
        # img_response = self._drone_client.simGetImages([airsim.ImageRequest("front_center_custom", airsim.ImageType.DepthPerspective,
        #                                             pixels_as_float=True, compress=False)],vehicle_name = self.drone_name)[0]
        img_response = self._drone_client.simGetImages(
                [airsim.ImageRequest('front_center_custom', airsim.ImageType.Scene, pixels_as_float=False, compress=False)],vehicle_name = self.drone_name)[0]
        img1d = np.array(img_response.image_data_float, dtype=np.float)
        img1d[img1d > 255] = 255
        img2d = np.reshape(img1d, (img_response.height, img_response.width))
        position = np.array([img_response.camera_position.x_val,img_response.camera_position.y_val,img_response.camera_position.z_val])
        return img2d, position, img_response.time_stamp  

    def _update_map(self):
        depth,position,self.last_time_stamp = self._get_depth_img()
        self.grid_map.update_map(depth,position)

    def _cal_tollerate(self):
        pass

    def rescure(self):
        print(self._get_collision_state())
        pass

    def random_action(self):
        return random.randint(0,len(self._action_set))

    def action_to_control(self, action_index):
        # TODO Add jerk/acc/position mode
        cur_action = self._action_set[action_index]
        if self.action_mode == 'velocity':
            cur_action = cur_action*self.vel_ratio
            cur_action[3] = cur_action[3]/self.vel_ratio*self.vel_yaw_ratio
            self._drone_client.moveByVelocityAsync(cur_action[0],cur_action[1],cur_action[2],self.frame_time,
                                                    yaw_mode = airsim.YawMode(is_rate = True, yaw_or_rate = cur_action[3]))
    
    def step_synchronous(self, action_ind: int):
        pass

    def step_asynchronous(self, action_ind: int):
        self._drone_client.simPause(False)
        self.action_to_control(action_ind)
        time.sleep(self.frame_time/10**9)
        self._update_map()
        self._drone_client.simPause(True)
        pass

    def seed(self, seed: Optional[int] = None):
        pass
    
    def step(self, action_ind: int):
        if self.step_mode == 'asynchronous':
            self.step_asynchronous(action_ind)
        else:
            self.step_synchronous(action_ind)
        done = False
        info = 'None'
        collisionInfo = self._get_collision_state()
        if collisionInfo.has_collided:
            reward = -1
            self.rescure()
            done = False
            info = 'Collision!!!'
        else:
            # TODO Calculate map_update_rate and energy loss
            known_region = len(np.where(self.grid_map.global_map != 1)[0])
            map_update_grid = known_region - self.last_known_region
            energy_loss = 0
            reward = map_update_grid/100 - energy_loss
            done = True
            info = "map_update_grid: " + str(map_update_grid)
        if self.rend:
            self.update_viwer()
        return self.grid_map.local_map, reward, done, info
        
    def reset(self):
        self._drone_client.reset() # or just set position to orgion
        pass

    def close(self) -> None:
        """
        Cleanup any leftovers by the environment
        """
        pass

    def render(self, mode: str):
        self.rend = True
        plt.rcParams["figure.figsize"] = [7.00, 3.50]
        plt.rcParams["figure.autolayout"] = True
        self.ax = plt.figure().add_subplot(projection='3d')
        self.update_viwer()
        
    def update_viwer(self):
        facecolors = np.where(self.grid_map.global_map == 0, '#FFAAAAAA', self.grid_map.global_map)
        facecolors = np.where(facecolors == 1, '#00000000', facecolors)
        facecolors = np.where(facecolors == 2, '#FFFF7F00', facecolors)
        facecolors = np.where(facecolors == 3, '#FF555555', facecolors)
        filled = np.ones(self.grid_map.global_map_shape)
        x, y, z = np.indices(np.array(filled.shape)).astype(float)
        self.ax.voxels(x, y, z, filled, facecolors=facecolors, edgecolors=facecolors)

    def manual_control(self):
        #start joystick control thread
        pass

        