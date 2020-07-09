"""
Containability Imagination for an object (urdf specified)

Author: Hongtao Wu
Mar 10, 2020
"""

import os
import time
import numpy as np
from containability.containability_3_1 import Containability

map_benchmark = True
content_urdf = "/home/hongtao/Dropbox/ICRA2021/data/general/m&m.urdf"
obj_dir = "/home/hongtao/Dropbox/ICRA2021/data/test_set_all"
result_dir = "/home/hongtao/Dropbox/ICRA2021/benchmark/test_set_all_imgn_wo_rotation_translation_0708"
obj_list = os.listdir(obj_dir)
# obj_list=["Amazon_Name_Card_Holder"]

print "Object number: ", len(obj_list)

for obj_name in obj_list:
    start_time = time.time()
    obj_urdf = os.path.join(obj_dir, obj_name, obj_name + "_mesh_0.urdf")
    obj_vhacd_mesh = os.path.join(obj_dir, obj_name, obj_name + "_mesh_0_vhacd.obj")
    C = Containability(obj_urdf, obj_vhacd_mesh, rotate=False, translate=False, friction=True, restitution=True,
        obj_zero_pos=[0, 0, 1], obj_zero_orn=[0, 0, 0], check_process=False, mp4_dir=None, object_name=obj_name, content_urdf=content_urdf)

    containability_affordance, sphere_in_percentage = C.get_containability()
    
    if not map_benchmark:
        if containability_affordance:
            drop_spot = C.find_drop_center()
            print("Dropping at: {}".format(drop_spot))
        else:
            drop_spot = [np.nan, np.nan, np.nan]
    
    C.disconnect_p()

    imagination_time = time.time() - start_time

    result_txt_name = os.path.join(result_dir, obj_name + ".txt")
    if not map_benchmark:
        with open(result_txt_name, "w") as file1:
            today = time.strftime("%Y-%m-%d-%H-%M", time.localtime())
            file1.write("Name: " + obj_name + "\n")
            file1.write("Date: " + today + "\n")
            file1.write("Content: " + content_urdf.split('/')[-1].split('.')[0] + "\n")
            file1.write("Containability: " + str(containability_affordance) + "\n")
            file1.write("Sphere in percentage: " + str(sphere_in_percentage) + "\n")
            file1.write("Pour position: " + str(list(drop_spot)) + "\n")
            file1.write("Containability imagination time: " + str(imagination_time) + "\n")
            file1.write("Object url: \n")
    
        # print("------------------")
        # print(obj_name)
        # print("Containability: {}".format(containability_affordance))
        # print("Sphere Percent: {}".format(sphere_in_percentage))
    else:
        with open(result_txt_name, "w") as file1:
            writerow = 'container ' + str(sphere_in_percentage) + ' 0 1 2 3'
            file1.write(writerow)