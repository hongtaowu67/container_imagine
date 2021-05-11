# Main script for containability magination with real robot experiments:
#     1. Capture views of the object
#     2. Reconstruct the object with TSDF fusion
#     3. Process the object with VHACD for convex decomposition
#     4. Containability imagination
#     5. Pouring imagination
#     5. Pour beads into the object with a cup

# Author: Hongtao Wu
# Institution: Johns Hopkins University
# Date: June 10, 2020

import os
import numpy as np

from capture_view import AutoCapture
from processing.tsdf_fusion import run_tsdf_fusion, tsdf_fusion_postprocess
from processing.utils import run_vhacd, write_urdf
from containability import Containability
from pouring import CupPour
from pick_and_pour import PickAndPour

parser = argparse.ArgumentParser(description="Real robot pouring experiment")
parser.add_argument("root_dir",
                    type=str,
                    help="root directory of the container imagination")
parser.add_argument("data_root_dir",
                    type=str,
                    help="root directory of the data")
parser.add_argument("data_name", type=str, help="name of the scene captured")
parser.add_argument("vhacd_dir", type=str, help="source directory of V-HACD")
parser.add_argument("-p",
                    "--pour",
                    type=bool,
                    default=False,
                    help="If set as True, pouring imagination is activated")
parser.add_argument("-r",
                    "--robotIP",
                    type=str,
                    default=None,
                    help="The ip address of the UR5 robot")
parser.add_argument(
    "-v",
    "--visualize",
    type=bool,
    default=False,
    help="If set as True, visualization from Pybullet will be activated")

parser.add_argument(
    "-m",
    "--mp4",
    type=str,
    default=None,
    help=
    "If a directory is given, the video of the imagination will be saved to that directory. Need to used with visualization (-v)"
)

args = parser.parse_args()

# Pouring
pouring = args.pour
robot_ip = args.robotIP
if pouring:
    print("Pouring imagination and robot pouring is activated...")
    print("Robot IP address: {}".format(robot_ip))

# Visualization and MP4
visualization = args.visualize
if visualization:
    print("Visualization is activated...")

mp4_dir = args.mp4
if mp4_dir:
    print("Imagination Video will be saved in {}".format(mp4_dir))

root_dir = args.root_dir
cup_urdf = os.path.join(root_dir, "object/Cup_GeoCenter.urdf")
content_urdf = os.path.join(root_dir, "object/m&m.urdf")
marker_content_urdf = os.path.join(root_dir, "object/m&m_red.urdf")

vhacd_dir = args.vhacd

data_root_dir = args.data_root_dir
data_name = args.data_name

data_dir = os.path.join(data_root_dir, data_name)
if not os.path.exists(data_dir):
    os.mkdir(data_dir)
    os.mkdir(os.path.join(data_dir, 'rgbd'))

# Extrinsic camera calibration file
cam2ee_file = os.path.join(root_dir, "calibrate/camera_pose.txt")

############### Capture views of the object #################
AC = AutoCapture(data_folder=os.path.join(data_dir, 'rgbd'),
                 acc=1.0,
                 vel=1.0,
                 cam2ee_file=cam2ee_file)
AC.collect_data()

######## 3D reconstruct the object with TSDF Fusion #########
tsdf_fusion_dir = os.path.join(root_dir, 'processing/TSDFfusion')

# TSDF Fusion

camera_intrinsics_file = os.path.join(root_dir,
                                      "calibrate/camera-intrinsics.txt")
run_tsdf_fusion(tsdf_fusion_dir,
                data_dir,
                camera_intrinsics_file,
                voxel_grid_origin_x=-0.3,
                voxel_grid_origin_y=-0.55,
                voxel_grid_origin_z=0.03,
                fast_tsdf_settings=True)

# Segementation
tsdf_bin_file = os.path.join(data_root_dir, data_name, 'rgbd/tsdf.bin')
tsdf_ply_file = os.path.join(data_root_dir, data_name, 'rgbd/tsdf.ply')
ply_output_prefix = os.path.join(data_root_dir, data_name,
                                 data_name + '_point')
obj_mesh_output_prefix = os.path.join(data_root_dir, data_name,
                                      data_name + '_mesh')
tsdf_fusion_postprocess(tsdf_bin_file, tsdf_ply_file, ply_output_prefix,
                        obj_mesh_output_prefix)

##################### VHACD processing #######################
# VHACD
object_name = data_name + "_mesh_0"
# This follows the directory struction from https://github.com/kmammou/v-hacd
vhacd_executable_dir = os.path.join(vhacd_dir, "src/build/test")
input_file = os.path.join(data_root_dir, data_name, object_name + '.obj')
output_file = os.path.join(data_root_dir, data_name,
                           object_name + '_vhacd.obj')
run_vhacd(vhacd_executable_dir, input_file, output_file)

# URDF file
obj_urdf = os.path.join(data_root_dir, data_name, object_name + '.urdf')
obj_original_file = object_name + '.obj'
obj_vhacd_file = object_name + '_vhacd.obj'
write_urdf(obj_urdf, obj_original_file, obj_vhacd_file)

################# Containability Imagination #################
print("Start containability imagination...")
obj_vhacd_path = os.path.join(data_root_dir, data_name, obj_vhacd_file)

print('URDF: ', obj_urdf)

C = Containability(obj_urdf,
                   obj_vhacd_path,
                   obj_zero_pos=[0, 0, 1],
                   obj_zero_orn=[0, 0, 0],
                   check_process=True,
                   mp4_dir=mp4_dir,
                   object_name=object_name,
                   content_urdf=content_urdf)

containability_affordance, sphere_in_percentage = C.get_containability()
sphere_in_list = np.array(C.sphere_in_drop_pos)

C.disconnect_p()

if containability_affordance:
    drop_spot = C.find_drop_center()
    print("Dropping at: {}".format(drop_spot))
else:
    drop_spot = [np.nan, np.nan, np.nan]

################### Pouring Imagination ######################
if pouring:
    if containability_affordance:
        print("Start pouring imagination...")
        sphere_in_list_se2 = sphere_in_list[:, :2]
        CP = CupPour(cup_urdf,
                     content_urdf,
                     obj_urdf,
                     drop_spot,
                     sphere_in_list_se2,
                     indent_num=3,
                     content_num=60,
                     obj_zero_pos=[0, 0, 1],
                     check_process=True,
                     mp4_dir=mp4_dir,
                     object_name=object_name)
        spill_list = CP.cup_pour()
        print("Spill List: {}".format(spill_list))

        imagined_pour_pos, imagined_cup_angle = CP.best_pour_pos_orn()
        CP.disconnect_p()
    else:
        spill_list = [[np.nan, np.nan, np.nan], [np.nan, np.nan, np.nan],
                      [np.nan, np.nan, np.nan], [np.nan, np.nan, np.nan],
                      [np.nan, np.nan, np.nan], [np.nan, np.nan, np.nan],
                      [np.nan, np.nan, np.nan], [np.nan, np.nan, np.nan]]

################## Real Robot Pouring #####################
if pouring:
    if containability_affordance:
        PP = PickAndPour(robot_ip=robot_ip, acc=0.5, vel=0.5)
        PP.pick_vertical()
        PP.pour_multi_orn(imagined_pour_pos, bottle_angle=imagined_cup_angle)