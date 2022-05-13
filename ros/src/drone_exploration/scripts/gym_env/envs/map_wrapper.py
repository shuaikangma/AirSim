import numpy as np
import math

import yaml


# STEP 1 坐标转换，用一个角点作为原点
# STEP 2 区域离散化，将空间变成矩阵代表的grid
# STEP 3 搞清楚插入的点云numpy矩阵是什么东西 -- 以传感器光心为原点，垂直传感器建立坐标系的h*w个三维点坐标，如果depth为nan，则坐标为nan
# STEP 3.5 建立临时的地图
# STEP 4 插入点云(增加对应区域为occupy的概率) -- 参考如何概率插入点云地图
# STEP 5 对应插入的occupy与传感器位置连线设为空，降低此处为occupy的概率
# STEP 6 合并临时地图和全局地图

# TODO 坐标变换
# 全局坐标到地图坐标
# 相机坐标系到全局坐标系

def depth_dedistortion(PointDepth, f):
    H = PointDepth.shape[0]
    W = PointDepth.shape[1]
    i_c = float(H) / 2 - 1
    j_c = float(W) / 2 - 1
    columns, rows = np.meshgrid(np.linspace(0, W-1, num=W), np.linspace(0, H-1, num=H))
    DistanceFromCenter = ((rows - i_c)**2 + (columns - j_c)**2)**(0.5)
    PlaneDepth = PointDepth / (1 + (DistanceFromCenter / f)**2)**(0.5)
    return PlaneDepth

def generate_pointcloud(depth,fx, fy, cx, cy):
    """convert the depth image into pointcloud
       Notice: the unit of the coordinates is same as 1000 * z / 256 of depth image
    Args:
        depth (np.array): shape w*h
        fx (float): Camera internal parameter
        fy (float): Camera internal parameter
        cx (float): Camera internal parameter
        cy (float): Camera internal parameter
    Returns:
        np.array: shape w*h*3, Cartesian coordinates of the pointcloud
    """
    rows, cols = depth.shape
    c, r = np.meshgrid(np.arange(cols), np.arange(rows), sparse=True)
    valid = (depth > 0) & (depth < 255)
    z = 1000 * np.where(valid, depth / 256.0, np.nan)
    x = np.where(valid, z * (c - cx) / fx, 0)
    y = np.where(valid, z * (r - cy) / fy, 0)
    return np.dstack((x, y, z))

def pointcloud_from_depth(depth, fx, fy, cx, cy):
    assert depth.dtype.kind == "f", "depth must be float and have meter values"
    rows, cols = depth.shape
    c, r = np.meshgrid(np.arange(cols), np.arange(rows), sparse=True)
    valid = ~np.isnan(depth)
    z = np.where(valid, depth, np.nan)
    x = np.where(valid, z * (c - cx) / fx, np.nan)
    y = np.where(valid, z * (r - cy) / fy, np.nan)
    pc = np.dstack((x, y, z))
    return pc

def quaternion_to_euler(quaternionr):
    """convert quaternion to euler angle of rpy

    Args:
        quaternionr (airsim.Quaternionr()): x/y/z/w_val 

    Returns:
        float: r,p,y degree
    """
    x = quaternionr.x_val
    y = quaternionr.y_val
    z = quaternionr.z_val
    w = quaternionr.w_val
    r = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    r = r / math.pi * 180
    p = math.asin(2 * (w * y - z * x))
    p = p / math.pi * 180
    y = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    y = y / math.pi * 180
    return r,p,y


class GridMap():
    def __init__(self,config_path: str = '/home/mrmmm/DRL_Exploration_With_Airsim/ros/src/drone_exploration/scripts/gym_env/configs/env_config.yaml') -> None:
        yaml_instream = open(config_path,'r',encoding='utf-8')
        yaml_config = yaml_instream.read()
        config = yaml.safe_load(yaml_config)
        self.camera_pram = config['drone']['camera_pram']
        self.resolution = config['map']['resolution']
        self.world_orgin = np.array(config['map']['world_orgin'])
        self.global_range = config['map']['global_map_range']
        self.loacl_map_range = config['map']['loacl_map_range']
        self.global_map_shape = [int(f/self.resolution) for f in self.global_range]
        self.global_map = np.ones(self.global_map_shape,dtype=np.uint8)
        self.loacl_map_shape = [int(f/self.resolution) for f in self.loacl_map_range]
        self.local_map = np.ones(self.loacl_map_shape,dtype=np.uint8) 
        #self.tmp_map = np.ones(self.shape,dtype=uint8) 
        yaml_instream.close()

    def _gene_occupy_grid(self, pointcloud, position_coordinate):
        #self.tmp_map = np.ones(self.shape,dtype=uint8) 
        #depth = depth_dedistortion(depth,self.camera_pram.fx)
        #pointcloud = generate_pointcloud(depth, self.camera_pram.fx, self.camera_pram.fy, self.camera_pram.cx, self.camera_pram.cy)
        #pointcloud = pointcloud.squeeze()
        for coordinate in pointcloud:
            xyz = (coordinate/self.resolution + position_coordinate).around.astype(int)
            self.global_map[xyz[0]][xyz[1]][xyz[2]] = 0
            # x = int(coordinate[0]/self.resolution) + position_coordinate[0]
            # y = int(coordinate[1]/self.resolution) + position_coordinate[1]
            # z = int(coordinate[2]/self.resolution) + position_coordinate[2]
            # self.global_map[x][y][z] = 3          

    def _update_scan_free(self, pointcloud, position_coordinate):
        step = 0.25
        for coordinate in pointcloud:
            depth_value = (coordinate[0]**2 + coordinate[1]**2 + coordinate[2]**2)**0.5
            direction = coordinate/depth_value
            step_depth = step * self.resolution
            while step_depth < depth_value :
                xyz = (direction*step_depth/self.resolution + position_coordinate).around.astype(int)
                # x = int(direction[0]*step_depth) + position_coordinate[0]
                # y = int(direction[1]*step_depth) + position_coordinate[1]
                # z = int(direction[2]*step_depth) + position_coordinate[2]
                if self.global_map[xyz[0]][xyz[1]][xyz[2]] == 1:
                    self.global_map[xyz[0]][xyz[1]][xyz[2]] = 0
                step_depth += step * self.resolution

    def update_map(self,depth, position):
        #self.tmp_map = np.ones(self.shape,dtype=uint8) 
        position_coordinate = ((position+self.world_orgin)/self.resolution).around.astype(int)
        depth = depth_dedistortion(depth,self.camera_pram.fx)
        pointcloud = generate_pointcloud(depth, self.camera_pram.fx, self.camera_pram.fy, self.camera_pram.cx, self.camera_pram.cy)
        pointcloud = pointcloud.squeeze()
        self._gene_occupy_grid(pointcloud, position_coordinate)
        self._update_scan_free(pointcloud, position_coordinate)
        self.global_map[position_coordinate[0]][position_coordinate[1]][position_coordinate[2]] = 2
        region_start = (position_coordinate - self.loacl_map_shape/2).around.astype(int)
        region_end = region_start + self.loacl_map_shape
        self.local_map = self.global_map[region_start[0]:region_end[0]][region_start[1]:region_end[1]][region_start[2]:region_end[2]]


def main():
    pass


if __name__ == "__main__":
    main()