from pickletools import uint4, uint8
import glooey
import imgviz
import numpy as np
import pyglet
import trimesh
import trimesh.transformations as tf
import trimesh.viewer
import math


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

def generatepointcloud(depth):
    rows, cols = depth.shape
    c, r = np.meshgrid(np.arange(cols), np.arange(rows), sparse=True)
    valid = (depth > 0) & (depth < 255)
    z = 1000 * np.where(valid, depth / 256.0, np.nan)
    x = np.where(valid, z * (c - Cx) / Fx, 0)
    y = np.where(valid, z * (r - Cy) / Fy, 0)
    return np.dstack((x, y, z))

def pointcloud_from_depth(depth, fx, fy, cx, cy):
    """convert the depth image into pointcloud
       Notice: the unit of the coordinates is same as z of depth image

    Args:
        depth (np.array): shape w*h
        fx (float): Camera internal parameter
        fy (float): Camera internal parameter
        cx (float): Camera internal parameter
        cy (float): Camera internal parameter

    Returns:
        np.array: shape w*h*3, Cartesian coordinates of the pointcloud
    """
    assert depth.dtype.kind == "f", "depth must be float and have meter values"
    rows, cols = depth.shape
    c, r = np.meshgrid(np.arange(cols), np.arange(rows), sparse=True)
    valid = ~np.isnan(depth)
    z = np.where(valid, depth, np.nan)
    x = np.where(valid, z * (c - cx) / fx, np.nan)
    y = np.where(valid, z * (r - cy) / fy, np.nan)
    pc = np.dstack((x, y, z))
    return pc

def depthConversion(PointDepth, f):
    H = PointDepth.shape[0]
    W = PointDepth.shape[1]
    i_c = float(H) / 2 - 1
    j_c = float(W) / 2 - 1
    columns, rows = np.meshgrid(np.linspace(0, W-1, num=W), np.linspace(0, H-1, num=H))
    DistanceFromCenter = ((rows - i_c)**2 + (columns - j_c)**2)**(0.5)
    PlaneDepth = PointDepth / (1 + (DistanceFromCenter / f)**2)**(0.5)
    return PlaneDepth
    
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


class grid_map():
    def __init__(self,range,resolution,world_orgin) -> None:
        self.resolution = resolution
        self.world_orgin = world_orgin
        self.shape = (int(range[0]/resolution), int(range[1]/resolution), int(range[2]/resolution))
        self.global_map = np.ones(self.shape,dtype=uint8)
        self.tmp_map = np.ones(self.shape,dtype=uint8) 
        pass

    def gene_occupy_grid(depth,)


def main():
    pass


if __name__ == "__main__":
    main()
