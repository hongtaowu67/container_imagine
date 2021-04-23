# Main script for Containability Imagination:
# 1. Containability imagination
# 2. Pouring imagination
# This code does not involve real robot. It imagines the containability of the object
# given the object model. For real robot experiments, please refer to main_real_robot.py

# Author: Hongtao Wu
# Institution: Johns Hopkins University
# Date: June 10, 2020

from __future__ import print_function

import os
import time
from datetime import date

import numpy as np

from containability import Containability
from pour.pouring import CupPour

# Imagine pouring or not
pouring = True

root_dir = os.getcwd()
cup_urdf = os.path.join(root_dir, "object/Cup_GeoCenter.urdf")
content_urdf = os.path.join(root_dir, "object/m&m.urdf")
data_root_dir = "/home/hongtao/Dropbox/ICRA2021/data"

data_name = "Juvale_Chef_Hat_pour_pca"
mesh_name = data_name + "_mesh_0"
obj_vhacd_file = mesh_name + "_vhacd.obj"

start_time = time.time()

data_folder = os.path.join(data_root_dir, data_name)
obj_urdf = os.path.join(data_folder, mesh_name + ".urdf")

################# Containability Imagination #################
print("Start containability imagination...")
obj_vhacd_path = os.path.join(data_root_dir, data_name, obj_vhacd_file)

mp4_dir = os.path.join(data_root_dir, data_name)
print('URDF: {}'.format(obj_urdf))

C = Containability(obj_urdf,
                   obj_vhacd_path,
                   obj_zero_pos=[0, 0, 1],
                   obj_zero_orn=[0, 0, 0],
                   check_process=False,
                   mp4_dir=mp4_dir,
                   object_name=mesh_name,
                   content_urdf=content_urdf)

containability_affordance, sphere_in_percentage = C.get_containability()
sphere_in_list = np.array(C.sphere_in_drop_pos)

C.disconnect_p()

if containability_affordance:
    drop_spot = C.find_drop_center()
    print("Dropping at: {}".format(drop_spot))
else:
    drop_spot = [np.nan, np.nan, np.nan]

containability_imagination_time = time.time() - start_time
#################################################################

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
                     check_process=False,
                     mp4_dir=mp4_dir,
                     object_name=mesh_name)
        spill_list = CP.cup_pour()
        print("Spill List: {}".format(spill_list))

        imagined_pour_pos, imagined_cup_angle = CP.best_pour_pos_orn()
        CP.disconnect_p()
    else:
        spill_list = [[np.nan, np.nan, np.nan], [np.nan, np.nan, np.nan],
                      [np.nan, np.nan, np.nan], [np.nan, np.nan, np.nan],
                      [np.nan, np.nan, np.nan], [np.nan, np.nan, np.nan],
                      [np.nan, np.nan, np.nan], [np.nan, np.nan, np.nan]]
else:
    spill_list = [[np.nan, np.nan, np.nan], [np.nan, np.nan, np.nan],
                  [np.nan, np.nan, np.nan], [np.nan, np.nan, np.nan],
                  [np.nan, np.nan, np.nan], [np.nan, np.nan, np.nan],
                  [np.nan, np.nan, np.nan], [np.nan, np.nan, np.nan]]

pouring_imagination_time = time.time(
) - start_time - containability_imagination_time
##############################################################

# Imagination with pouring
#################################################################
if pouring:
    result_txt_name = os.path.join(data_folder, data_name + ".txt")
    with open(result_txt_name, "w") as file1:
        today = time.strftime("%Y-%m-%d-%H-%M", time.localtime())
        file1.write("Name: " + data_name + "\n")
        file1.write("Date: " + today + "\n")
        file1.write("Containability: " + str(containability_affordance) + "\n")
        file1.write("Sphere in percentage: " + str(sphere_in_percentage) +
                    "\n")
        file1.write("Average drop position: " + str(list(drop_spot)) + "\n")
        file1.write("Imagined pour position: " + str(imagined_pour_pos) + "\n")
        file1.write("Imagined cup angle: " + str(imagined_cup_angle) + "\n")
        file1.write("Spill List: " + str(list(spill_list)) + "\n")
        file1.write("Containability imagination time: " +
                    str(containability_imagination_time) + "\n")
        file1.write("Pouring imagination time: " +
                    str(pouring_imagination_time) + "\n")
#################################################################

# Imagination without pouring
#################################################################
if not pouring:
    result_txt_name = os.path.join(data_folder, data_name + ".txt")
    with open(result_txt_name, "w") as file1:
        today = time.strftime("%Y-%m-%d-%H-%M", time.localtime())
        file1.write("Name: " + data_name + "\n")
        file1.write("Date: " + today + "\n")
        file1.write("Containability: " + str(containability_affordance) + "\n")
        file1.write("Sphere in percentage: " + str(sphere_in_percentage) +
                    "\n")
        file1.write("Pour position: " + str(list(drop_spot)) + "\n")
        file1.write("Data processing time: " + str(data_preprocessing_time) +
                    "\n")
        file1.write("Containability imagination time: " +
                    str(containability_imagination_time) + "\n")
#################################################################