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
from datetime import date

import numpy as np

from capture_view_pick import AutoCapture
from tsdf_fusion_segmentation import run_tsdf_fusion_cuda, segment_tsdf_fast
from processing.process import run_vhacd, write_urdf
from containability.containability_3_1 import Containability
from pick_and_pour_2 import PickAndPour

content_urdf = "/home/hongtao/Dropbox/ICRA2021/data/general/m&m.urdf"
data_name = "Juvale_Chef_Hat_jb"
pouring = True

data_root_dir = "/home/hongtao/Dropbox/ICRA2021/data"


start_time = time.time()

root_dir = os.getcwd()

data_folder = os.path.join(data_root_dir, data_name)
if not os.path.exists(data_folder):
    os.mkdir(data_folder)
    os.mkdir(os.path.join(data_folder, 'rgbd'))

# Extrinsic camera calibration file    
cam2ee_file = os.path.join(root_dir, "calibrate/camera_pose.txt")

############### Capture views of the object #################
AC = AutoCapture(data_folder=os.path.join(data_folder, 'rgbd'), 
                    acc=1.0, vel=1.0, cam2ee_file=cam2ee_file)
AC.collect_data()

autocapture_time = time.time() - start_time
#############################################################


######## 3D reconstruct the object with TSDF Fusion #########
tsdf_fusion_dir = os.path.join(root_dir, 'reconstruction/tsdf-fusion')

# TSDF Fusion
image_folder = os.path.join(data_root_dir, data_name, 'rgbd')
camera_intrinsics_file = os.path.join(root_dir, "calibrate/camera-intrinsics.txt")
run_tsdf_fusion_cuda(tsdf_fusion_dir, image_folder, camera_intrinsics_file, 
    voxel_grid_origin_x=-0.3, voxel_grid_origin_y=-0.55, voxel_grid_origin_z=0.03, fast_tsdf_settings=True)

# Segementation
tsdf_bin_file = os.path.join(data_root_dir, data_name, 'rgbd/tsdf.bin')
tsdf_ply_file = os.path.join(data_root_dir, data_name, 'rgbd/tsdf.ply')
ply_output_prefix = os.path.join(data_root_dir, data_name, data_name + '_point')
obj_mesh_output_prefix = os.path.join(data_root_dir, data_name, data_name + '_mesh')
segment_tsdf_fast(tsdf_bin_file, tsdf_ply_file, ply_output_prefix, obj_mesh_output_prefix)
##############################################################


##################### VHACD processing #######################
object_name = data_name + "_mesh_0"

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

preprocessing_time = time.time() - start_time - autocapture_time
##############################################################


################# Containability Imagination #################
object_name = data_name + "_mesh_0"
obj_urdf = os.path.join(data_root_dir, data_name, object_name + '.urdf')
obj_vhacd_mesh = os.path.join(data_root_dir, data_name, object_name + "_mesh_0_vhacd.obj")

mp4_dir = os.path.join(data_root_dir, data_name)
print('URDF: ', obj_urdf)

C = Containability(obj_urdf, obj_vhacd_mesh, obj_zero_pos=[0, 0, 1], obj_zero_orn=[0, 0, 0], 
        check_process=True, mp4_dir=mp4_dir, object_name=object_name, content_urdf=content_urdf)

containability_affordance, sphere_in_percentage = C.get_containability()

if containability_affordance:
    drop_spot = C.find_drop_center()
    print("Dropping at: {}".format(drop_spot))
else:
    drop_spot = [np.nan, np.nan, np.nan]

imagination_time = time.time() - start_time - autocapture_time - preprocessing_time
#################################################################

if pouring:
    if containability_affordance:
        PP = PickAndPour(acc=1.0, vel=1.0)
        PP.pick()
        PP.pour(drop_spot)
    pouring_time = time.time() - start_time - autocapture_time - preprocessing_time - imagination_time
else:
    pouring_time = np.nan

#################################################################
result_txt_name = os.path.join(data_folder, data_name + ".txt")
with open(result_txt_name, "w") as file1:
    today = time.strftime("%Y-%m-%d-%H-%M", time.localtime())
    file1.write("Name: " + data_name + "\n")
    file1.write("Date: " + today + "\n")
    file1.write("Containability: " + str(containability_affordance) + "\n")
    file1.write("Sphere in percentage: " + str(sphere_in_percentage) + "\n")
    file1.write("Pour position: " + str(list(drop_spot)) + "\n")
    file1.write("Robot scanning time: " + str(autocapture_time) + "\n")
    file1.write("Model processing time: " + str(preprocessing_time) + "\n")
    file1.write("Containability imagination time: " + str(imagination_time) + "\n")
    file1.write("Pouring time: " + str(pouring_time) + "\n")
    file1.write("Object url: \n")
#################################################################