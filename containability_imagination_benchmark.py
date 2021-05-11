# Containability Imagination Benchmark experiment
# The result of the containability imagination for each object in the <data_root_dir> will
# be saved in the <result_dir> in a format of:
#   "container sphere-in-percentaget 0 1 2 3"
# After the result is generated, run benchmark_map.py to see the result

# python containability_imagination_benchmark.py <root_dir> <data_root_dir> <result_dir>

# Author: Hongtao Wu
# Institution: Johns Hopkins University
# Date: Mar 10, 2020

from __future__ import print_function
import os
import time
import argparse
import numpy as np
from containability import Containability

parser = argparse.ArgumentParser(
    description="Containability imagination benchmark")
parser.add_argument("root_dir",
                    type=str,
                    help="root directory of the container imagination")
parser.add_argument("data_root_dir",
                    type=str,
                    help="root directory of the data")
parser.add_argument("result_dir",
                    type=str,
                    help="directory of for saving the result")

args = parser.parse_args()

root_dir = args.root_dir
obj_dir = args.data_root_dir
result_dir = args.result_dir

content_urdf = os.path.join(root_dir, "object/m&m.urdf")

obj_list = os.listdir(obj_dir)

print("Object number: ", len(obj_list))

for obj_name in obj_list:
    start_time = time.time()
    obj_urdf = os.path.join(obj_dir, obj_name, obj_name + "_mesh_0.urdf")
    obj_vhacd_mesh = os.path.join(obj_dir, obj_name,
                                  obj_name + "_mesh_0_vhacd.obj")
    C = Containability(obj_urdf,
                       obj_vhacd_mesh,
                       rotate=True,
                       translate=True,
                       friction=True,
                       restitution=True,
                       obj_zero_pos=[0, 0, 1],
                       obj_zero_orn=[0, 0, 0],
                       check_process=False,
                       mp4_dir=None,
                       object_name=obj_name,
                       content_urdf=content_urdf)

    containability_affordance, sphere_in_percentage = C.get_containability()

    C.disconnect_p()

    imagination_time = time.time() - start_time

    result_txt_name = os.path.join(result_dir, obj_name + ".txt")

    with open(result_txt_name, "w") as file1:
        writerow = 'container ' + str(sphere_in_percentage) + ' 0 1 2 3'
        file1.write(writerow)

print("Finish containability imagination benchmark!")