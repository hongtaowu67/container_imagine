"""
Real Object Containability Imagination:
    1. Capture views of the object
    2. Reconstruct the object with TSDF fusion
    3. Process the object with VHACD for convex decomposition
    4. Imagine the containability of the object
    5. Pour beads into the object with a bottle
"""

import os
import time

from capture_view_pick import AutoCapture
from tsdf_fusion_segmentation import run_tsdf_fusion_cuda, segment_tsdf_fast
from processing.process import run_vhacd, write_urdf
# from containability.containability_new import Containability
# from pick_and_pour import PickAndPour



root_dir = os.getcwd()

data_name = "GripperTest1_24view"
data_root_dir = "/home/hongtao/Dropbox/ICRA2021/data"

data_folder = os.path.join(data_root_dir, data_name)
if not os.path.exists(data_folder):
    os.mkdir(data_folder)
    os.mkdir(os.path.join(data_folder, 'rgbd'))

# Extrinsic camera calibration file    
cam2ee_file = os.path.join(root_dir, "calibrate/camera_pose.txt")

#### Capture views of the object ###
AC = AutoCapture(data_folder=os.path.join(data_folder, 'rgbd'), 
                    acc=1.0, vel=1.0, cam2ee_file=cam2ee_file)
AC.collect_data()
####################################


### 3D reconstruct the object with TSDF Fusion ###
tsdf_fusion_dir = os.path.join(root_dir, 'reconstruction/tsdf-fusion')

# TSDF Fusion
image_folder = os.path.join(data_root_dir, data_name, 'rgbd')
camera_intrinsics_file = os.path.join(root_dir, "calibrate/camera-intrinsics.txt")
run_tsdf_fusion_cuda(tsdf_fusion_dir, image_folder, camera_intrinsics_file, 
    voxel_grid_origin_x=-0.3, voxel_grid_origin_y=-0.55, voxel_grid_origin_z=0.03, fast_tsdf_settings=True)

# Segementation
tsdf_bin_file = os.path.join(data_root_dir, data_name, 'rgbd/tsdf.bin')
tsdf_ply_file = os.path.join(data_root_dir, data_name, 'rgbd/tsdf.ply')
ply_output_prefix = os.path.join(data_root_dir, data_name, data_name + '_point_debug')
obj_mesh_output_prefix = os.path.join(data_root_dir, data_name, data_name + '_mesh_debug')
segment_tsdf_fast(tsdf_bin_file, tsdf_ply_file, ply_output_prefix, obj_mesh_output_prefix)
####################################


### VHACD processing ###
start_time = time.time()
object_name = data_name + "_mesh_debug_0"

# VHACD
vhacd_dir = os.path.join(root_dir, 'processing')
input_file = os.path.join(data_root_dir, data_name, object_name + '.obj') 
output_file = os.path.join(data_root_dir, data_name, object_name + '_vhacd.obj')
run_vhacd(vhacd_dir, input_file, output_file)

# URDF file
urdf_path = os.path.join(data_root_dir, data_name, object_name + '.urdf')
obj_original_file = object_name + '.obj'
obj_vhacd_file = object_name + '_vhacd.obj'
write_urdf(urdf_path, obj_original_file, obj_vhacd_file)
process_time = time.time() - start_time
print("Total process time: ", process_time)
####################################


# ### Containability Imagination ###
# object_name = data_name + "_mesh_debug_0"
# obj_urdf = os.path.join(data_root_dir, data_name, object_name + '.urdf')

# mp4_dir = os.path.join(data_root_dir, data_name)
# print('URDF: ', obj_urdf)

# C = Containability(obj_urdf, obj_zero_pos=[0, 0, 1], obj_zero_orn=[0, 0, 0], 
#         check_process=True, mp4_dir=mp4_dir, object_name=object_name)

# containable_affordance, sphere_in_percentage = C.get_containability()

# if containable_affordance:
#     drop_spot = C.find_drop_center()
#     print("Dropping at: {}".format(drop_spot))
# else:
#     drop_spot = [np.nan, np.nan, np.nan]

# obj_containability_filename = data_name + '.txt'
# obj_containability_file = os.path.join(data_root_dir, data_name, obj_containability_filename)
# with open(obj_containability_file, 'w') as writefile:
#     write_line=[]
#     obj_info = "Name: " + data_name + '\n'
#     write_line.append(obj_info)
#     containability = "Containability: " + str(containable_affordance) + '\n'
#     write_line.append(containability)
#     sphere_percentage_line = "Sphere in percentage: " + str(sphere_in_percentage) + '\n'
#     write_line.append(sphere_percentage_line)
#     dropspot = "Drop Spot: " + str(list(drop_spot)) + '\n'
#     write_line.append(dropspot)
#     today = time.strftime("%Y-%m-%d-%H-%M", time.localtime())
#     time = "Time: " + today + '\n'
#     write_line.append(time)
#     writefile.writelines(write_line)

# C.disconnet()
# ####################################

# if containable_affordance:
#     PP = PickAndPour(acc=1.0, vel=1.0)
#     PP.pick()