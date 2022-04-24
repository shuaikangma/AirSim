import airsim
import binvox
import scipy
import octomap

def save_voxel_grid(filename,resolution):
    client = airsim.VehicleClient()
    center = airsim.Vector3r(0, 0, 0)
    is_sucess = client.simCreateVoxelGrid(center, 1, 1, 1, 0.5, filename)
    return is_sucess

def read_voxel_grid(filename, mode='dense'):
    return binvox.Binvox.read(filename, mode)

def read_show_voxel_grid(filename):
    model = read_voxel_grid(filename)
    print(model.data)
    print(model.scale)
    print(model.dims)
    scipy.ndimage.binary_dilation(model.data.copy(), output=model.data)

def merge_octomap():
    pass

if __name__ == "__main__":
    filename = '~/groundtruth.binvox'
    resolution = 0.5
    bo_ = save_voxel_grid(filename,resolution)
    if bo_:
        print('sucess')
    read_show_voxel_grid(filename)
    octomap.OcTree(0.1)