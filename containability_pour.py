"""
Containability Imagination for an object (urdf specified)

Author: Hongtao Wu
Mar 10, 2020
"""

import os
import time
import numpy as np
from containability.containability_3_1 import Containability
from pour.pouring_2 import BottlePour

content_urdf = "/home/hongtao/Dropbox/ICRA2021/data/general/m&m.urdf"
obj_dir = "/home/hongtao/Dropbox/ICRA2021/data/test_set_all"
# obj_dir = "/home/hongtao/Dropbox/ICRA2021/paper/Figure2"
result_dir = "/home/hongtao/Dropbox/ICRA2021/benchmark/0408_mm"
bottle_urdf = "/home/hongtao/Dropbox/ICRA2021/data/general/bottle/JuiceBottle_GeoCenter.urdf"
obj_list = os.listdir(obj_dir)
# obj_list=["Blue_Cup"]
print(obj_list)

for obj_name in obj_list:
    print("------------------")
    print(obj_name)

    start_time = time.time()
    obj_urdf = os.path.join(obj_dir, obj_name, obj_name + "_mesh_0.urdf")
    obj_vhacd_mesh = os.path.join(obj_dir, obj_name, obj_name + "_mesh_0_vhacd.obj")
    C = Containability(obj_urdf, obj_vhacd_mesh, obj_zero_pos=[0, 0, 1], obj_zero_orn=[0, 0, 0],
            check_process=False, mp4_dir=None, object_name=None, content_urdf=content_urdf)

    containability_affordance, sphere_in_percentage = C.get_containability()

    print("Containability: {}".format(containability_affordance))
    print("Sphere Percent: {}".format(sphere_in_percentage))
    
    if containability_affordance:
        pour_pos = C.find_drop_center()
        print "Dropping at: {}".format(pour_pos)
        C.disconnect_p()
        BP = BottlePour(bottle_urdf, content_urdf, obj_urdf, pour_pos, indent_num=3, content_num=60,
                obj_zero_pos=[0, 0, 1], check_process=False, mp4_dir=None, object_name=obj_name)
        spill_list = BP.bottle_pour()
        BP.disconnect_p()
        print "Spill List: {}".format(spill_list)
    else:
        pour_pos = [np.nan, np.nan, np.nan]
        spill_list = [np.nan, np.nan, np.nan, np.nan, np.nan, np.nan, np.nan, np.nan]
        C.disconnect_p()
    

    imagination_time = time.time() - start_time

    result_txt_name = os.path.join(result_dir, obj_name + ".txt")
    with open(result_txt_name, "w") as file1:
        today = time.strftime("%Y-%m-%d-%H-%M", time.localtime())
        file1.write("Name: " + obj_name + "\n")
        file1.write("Date: " + today + "\n")
        file1.write("Content: " + content_urdf.split('/')[-1].split('.')[0] + "\n")
        file1.write("Containability: " + str(containability_affordance) + "\n")
        file1.write("Sphere in percentage: " + str(sphere_in_percentage) + "\n")
        file1.write("Pour position: " + str(list(pour_pos)) + "\n")
        file1.write("Spill List: " + str(list(spill_list)) + "\n")
        file1.write("Containability imagination time: " + str(imagination_time) + "\n")
        file1.write("Object url: \n")