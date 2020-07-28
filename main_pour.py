"""
Real Object Containability Imagination:
    1. Capture views of the object
    2. Reconstruct the object with TSDF fusion
    3. Process the object with VHACD for convex decomposition
    4. Containability imagination
    5. Pouring imagination
    5. Pour beads into the object with a cup

Author: Hongtao Wu
June 10, 2020
"""

import os
import time
from datetime import date

import numpy as np

from capture_view_pick import AutoCapture
from tsdf_fusion_segmentation import run_tsdf_fusion_cuda, segment_tsdf_fast
from processing.process import run_vhacd, write_urdf
from containability.containability_3_1 import Containability
from pour.pouring_3 import CupPour
from pick_and_pour_3 import PickAndPour

cup_urdf = "/home/hongtao/Dropbox/ICRA2021/data/general/cup/Cup_GeoCenter.urdf"
content_urdf = "/home/hongtao/Dropbox/ICRA2021/data/general/m&m.urdf"
data_name = "TeChef_Egg_pan"
pouring = False

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
tsdf_fusion_dir = os.path.join(root_dir, 'reconstruction/TSDFfusion')

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
obj_urdf = os.path.join(data_root_dir, data_name, object_name + '.urdf')
obj_original_file = object_name + '.obj'
obj_vhacd_file = object_name + '_vhacd.obj'
write_urdf(obj_urdf, obj_original_file, obj_vhacd_file)

preprocessing_time = time.time() - start_time - autocapture_time
##############################################################


################# Containability Imagination #################
print "Start containability imagination..."
obj_vhacd_path = os.path.join(data_root_dir, data_name, obj_vhacd_file)

mp4_dir = os.path.join(data_root_dir, data_name)
print('URDF: ', obj_urdf)

C = Containability(obj_urdf, obj_vhacd_path, obj_zero_pos=[0, 0, 1], obj_zero_orn=[0, 0, 0], 
        check_process=True, mp4_dir=mp4_dir, object_name=object_name, content_urdf=content_urdf)

containability_affordance, sphere_in_percentage = C.get_containability()
C.disconnect_p()

if containability_affordance:
    drop_spot = C.find_drop_center()
    print("Dropping at: {}".format(drop_spot))
else:
    drop_spot = [np.nan, np.nan, np.nan]

containability_imagination_time = time.time() - start_time - autocapture_time - preprocessing_time
#################################################################


################### Pouring Imagination ######################
if pouring:
    if containability_affordance:
        print "Start pouring imagination..."
        BP = CupPour(cup_urdf, content_urdf, obj_urdf, drop_spot, indent_num=3, content_num=60,
                        obj_zero_pos=[0, 0, 1], check_process=True, mp4_dir=mp4_dir, object_name=object_name)
        spill_list = BP.cup_pour()
        BP.disconnect_p()
        print "Spill List: {}".format(spill_list)

        imagined_pour_pos, imagined_cup_angle = BP.best_pour_pos_orn()
    else:
        spill_list = [[np.nan, np.nan, np.nan], [np.nan, np.nan, np.nan], [np.nan, np.nan, np.nan],[np.nan, np.nan, np.nan],[np.nan, np.nan, np.nan],[np.nan, np.nan, np.nan],[np.nan, np.nan, np.nan],[np.nan, np.nan, np.nan]]
else:
    spill_list = [[np.nan, np.nan, np.nan], [np.nan, np.nan, np.nan], [np.nan, np.nan, np.nan],[np.nan, np.nan, np.nan],[np.nan, np.nan, np.nan],[np.nan, np.nan, np.nan],[np.nan, np.nan, np.nan],[np.nan, np.nan, np.nan]]

pouring_imagination_time = time.time() - start_time - autocapture_time - preprocessing_time - containability_imagination_time
##############################################################


################### Real Robot Pouring #####################
if pouring:
    if containability_affordance:
        PP = PickAndPour(acc=0.5, vel=0.5)
        PP.pick_vertical()
        PP.pour_multi_orn(imagined_pour_pos, bottle_angle=imagined_cup_angle)
    else:
        imagined_pour_pos = [np.nan, np.nan, np.nan]
        imagined_cup_angle = np.nan
    pouring_time = time.time() - start_time - autocapture_time - preprocessing_time - containability_imagination_time - pouring_imagination_time
else:
    imagined_pour_pos = [np.nan, np.nan, np.nan]
    imagined_cup_angle = np.nan
    pouring_time = np.nan
############################################################

# Containabililty with pouring
#################################################################
if pouring:
    result_txt_name = os.path.join(data_folder, data_name + ".txt")
    with open(result_txt_name, "w") as file1:
        today = time.strftime("%Y-%m-%d-%H-%M", time.localtime())
        file1.write("Name: " + data_name + "\n")
        file1.write("Date: " + today + "\n")
        file1.write("Containability: " + str(containability_affordance) + "\n")
        file1.write("Sphere in percentage: " + str(sphere_in_percentage) + "\n")
        file1.write("Average drop position: " + str(list(drop_spot)) + "\n")
        file1.write("Imagined pour position: " + str(imagined_pour_pos) + "\n")
        file1.write("Imagined cup angle: " + str(imagined_cup_angle) + "\n")
        file1.write("Spill List: " + str(list(spill_list)) + "\n")
        file1.write("Robot scanning time: " + str(autocapture_time) + "\n")
        file1.write("Model processing time: " + str(preprocessing_time) + "\n")
        file1.write("Containability imagination time: " + str(containability_imagination_time) + "\n")
        file1.write("Pouring imagination time: " + str(pouring_imagination_time) + "\n")
        file1.write("Pouring time: " + str(pouring_time) + "\n")
        file1.write("Object url: \n")
#################################################################

# Containability Imagination
#################################################################
if not pouring:
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
        file1.write("Containability imagination time: " + str(containability_imagination_time) + "\n")
        file1.write("Pouring time: " + str(pouring_time) + "\n")
        file1.write("Object url: \n")
#################################################################