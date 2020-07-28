"""
The pouring point is derived from affordance-net/tools/affnet_pouring_point.py
This code pour at the point. The cup angle is fixed.

Author: Hongtao Wu
July 07, 2020
"""

import os
import numpy as np 
import time

from pick_and_pour_3 import PickAndPour


data_dir = "/home/hongtao/Dropbox/ICRA2021/affnet_benchmark/affnet_benchmark_pouring_3DScanning"
obj_name = "Ikea_Framkalla_Mug_Pink_3D"
pour_point_txt = obj_name + "_affnet_pour_point.txt"
pour_point_z_txt = obj_name + "_affnet_pour_point_z.txt"
affnet_3DScanning = True

if not affnet_3DScanning:
    # Affnet pouring
    with open(os.path.join(data_dir, obj_name, pour_point_txt), "r") as f:
        p3D_str = f.readline()
        items = p3D_str.split(" ")
        p3D = np.zeros(3)
        for i in range(3):
            p3D[i] = float(items[i])

    z_offset = 0.10
    p3D[-1] += z_offset

else:
    # Affnet pouring with 3D Scanning
    with open(os.path.join(data_dir, obj_name, pour_point_txt), "r") as f:
        p3D_str = f.readline()
        items = p3D_str.split(" ")
        p3D = np.zeros(3)
        for i in range(2):
            p3D[i] = float(items[i])

    with open(os.path.join(data_dir, obj_name, pour_point_z_txt), "r") as f:
        z_str = f.readline()
        p3D[-1] = float(z_str)

print p3D

PP = PickAndPour(acc=0.5, vel=0.5)
time.sleep(2)
PP.pick_vertical()
PP.pour_multi_orn(p3D, bottle_angle=np.pi/4)