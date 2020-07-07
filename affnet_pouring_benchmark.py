"""
The pouring point is derived from affordance-net/tools/affnet_pouring_point.py
This code pour at the point. The cup angle is fixed.

Author: Hongtao Wu
July 07, 2020
"""

import os
import numpy as np 

from pick_and_pour_3 import PickAndPour


data_folder = "/home/hongtao/Dropbox/ICRA2021/affnet_benchmark/affnet_benchmark_pouring"
obj_name = "Ikea_Fargrik_Bowl"
pour_point_txt = obj_name + "_affnet_pour_point.txt"

with open(os.path.join(data_folder, obj_name, pour_point_txt), "r") as f:
    p3D_str = f.readline()
    items = p3D_str.split(" ")
    p3D = np.zeros(3)
    for i in range(3):
        p3D[i] = float(items[i])

z_offset = 0.1
p3D[-1] += z_offset
print p3D

PP = PickAndPour(acc=0.5, vel=0.5)
PP.pick_vertical()
PP.pour_multi_orn(p3D, bottle_angle=np.pi/4)