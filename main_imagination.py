# Main script for Containability Imagination:
# 1. Containability imagination
# 2. Pouring imagination
# This code does not involve real robot. It imagines the containability of the object
# given the object model. For real robot experiments, please refer to main_real_robot.py

# Author: Hongtao Wu
# Institution: Johns Hopkins University
# Date: June 10, 2020

# python main_imagination.py <root_dir> <data_dir> <data_name> <mesh_name> [-p] [-v] [-m]
# Use python main_imagination.py -h to see the option

from __future__ import print_function

import os
import time
import argparse
from datetime import date

import numpy as np

from containability import Containability
from pouring import CupPour

parser = argparse.ArgumentParser(description="Open containability imagination")
parser.add_argument("root_dir",
                    type=str,
                    help="root directory of the container imagination")
parser.add_argument("data_root_dir",
                    type=str,
                    help="root directory of the data")
parser.add_argument("data_name", type=str, help="name of the scene captured")
parser.add_argument("mesh_name",
                    type=str,
                    help="name of the mesh being imagined")
parser.add_argument("-p",
                    "--pour",
                    type=bool,
                    default=False,
                    help="If set as True, pouring imagination is activated")
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

# Imagine pouring or not
pour = args.pour
if pour:
    print("Pouring imagination is activated...")

# Visualization and MP4
visualization = args.visualize
if visualization:
    print("Visualization is activated...")

mp4_dir = args.mp4
if mp4_dir:
    print("Imagination Video will be saved in {}".format(mp4_dir))

# Root directory of the code repo
root_dir = args.root_dir
cup_urdf = os.path.join(root_dir, "object/Cup_GeoCenter.urdf")
content_urdf = os.path.join(root_dir, "object/m&m.urdf")
marker_content_urdf = os.path.join(root_dir, "object/m&m_red.urdf")

# Root directory of the data repo
data_root_dir = args.data_root_dir
# Data name (the data is captured as a scene and could contain more than one objects)
data_name = args.data_name
data_dir = os.path.join(data_root_dir, data_name)
# The mesh name in the data to be imagined
mesh_name = args.mesh_name
obj_vhacd_file = mesh_name + "_vhacd.obj"
obj_urdf = os.path.join(data_dir, mesh_name + ".urdf")

print('object urdf file: {}'.format(obj_urdf))

################# Containability Imagination #################
print("Start containability imagination on: {}".format(mesh_name))
obj_vhacd_path = os.path.join(data_root_dir, data_name, obj_vhacd_file)

C = Containability(obj_urdf,
                   obj_vhacd_path,
                   content_urdf=content_urdf,
                   obj_zero_pos=[0, 0, 1],
                   obj_zero_orn=[0, 0, 0],
                   check_process=visualization,
                   mp4_dir=mp4_dir,
                   object_name=mesh_name)

containability_affordance, sphere_in_percentage = C.get_containability()
sphere_in_list = np.array(C.sphere_in_drop_pos)

# # If you want to see the footprint of the containability result,
# # unconmment the following
# C.visualize_footprint(sphere_urdf=content_urdf,
#                       marker_sphere_urdf=marker_content_urdf)

C.disconnect_p()

if containability_affordance:
    drop_spot = C.find_drop_center()
    print("Pouring at: {}".format(drop_spot))

################### Pouring Imagination ######################
if pour:
    if containability_affordance:
        print("Start pouring imagination on: {}".format(mesh_name))
        sphere_in_list_se2 = sphere_in_list[:, :2]
        CP = CupPour(cup_urdf,
                     content_urdf,
                     obj_urdf,
                     drop_spot,
                     sphere_in_list_se2,
                     indent_num=3,
                     content_num=60,
                     obj_zero_pos=[0, 0, 1],
                     check_process=visualization,
                     mp4_dir=mp4_dir,
                     object_name=mesh_name)
        spill_list = CP.cup_pour()
        print("Spill List: {}".format(spill_list))

        imagined_pour_pos, imagined_cup_angle = CP.best_pour_pos_orn()
        CP.disconnect_p()