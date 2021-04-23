"""
Containability Imagination for an object (urdf specified)

Author: Hongtao Wu
Mar 10, 2020
"""

import os
import time
import csv
import numpy as np
from containability import Containability
from pour.pouring import CupPour

content_urdf = "/home/hongtao/Dropbox/ICRA2021/data/general/m&m.urdf"
obj_dir = "/home/hongtao/Dropbox/ICRA2021/data/test_set_containability"
# obj_dir = "/home/hongtao/Dropbox/ICRA2021/paper/Figure2"
# result_dir = "/home/hongtao/Dropbox/ICRA2021/benchmark/0616_mm_pour"
cup_urdf = "/home/hongtao/Dropbox/ICRA2021/data/general/cup/Cup_GeoCenter.urdf"
obj_list = os.listdir(obj_dir)
# obj_list=["Blue_Cup"]
print(obj_list)

containability_running_time_csv = "/home/hongtao/Dropbox/ICRA2021/benchmark/containability_running_time.csv"
pouring_running_time_csv = "/home/hongtao/Dropbox/ICRA2021/benchmark/pouring_running_time.csv"

for obj_name in obj_list:
    # print("------------------")
    # print(obj_name)

    containability_imagination_start_time = time.time()

    #####################
    # print "Start containability imagination..."
    obj_urdf = os.path.join(obj_dir, obj_name, obj_name + "_mesh_0.urdf")
    obj_vhacd_path = os.path.join(obj_dir, obj_name,
                                  obj_name + "_mesh_0_vhacd.obj")
    C = Containability(obj_urdf,
                       obj_vhacd_path,
                       obj_zero_pos=[0, 0, 1],
                       obj_zero_orn=[0, 0, 0],
                       check_process=False,
                       mp4_dir=None,
                       object_name=obj_name,
                       content_urdf=content_urdf)

    containability_affordance, sphere_in_percentage = C.get_containability()
    sphere_in_list = np.array(C.sphere_in_drop_pos)

    C.disconnect_p()

    # if containability_affordance:
    #     drop_spot = C.find_drop_center()
    #     print("Dropping at: {}".format(drop_spot))
    # else:
    #     drop_spot = [np.nan, np.nan, np.nan]

    containability_imagination_time = time.time(
    ) - containability_imagination_start_time

    with open(containability_running_time_csv, 'a') as f1:
        writer = csv.writer(f1)
        csvwriterow = []
        csvwriterow.append(obj_name)
        csvwriterow.append(containability_imagination_time)
        writer.writerow(csvwriterow)

######################
    if containability_affordance:
        pouring_imagination_start_time = time.time()
        drop_spot = C.find_drop_center()
        # print "Start pouring imagination..."
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
                     mp4_dir=None,
                     object_name=obj_name)
        spill_list = CP.cup_pour()
        # print "Spill List: {}".format(spill_list)

        imagined_pour_pos, imagined_cup_angle = CP.best_pour_pos_orn()

        CP.disconnect_p()
        pouring_imagination_time = time.time() - pouring_imagination_start_time
        with open(pouring_running_time_csv, 'a') as f2:
            writer = csv.writer(f2)
            csvwriterow = []
            csvwriterow.append(obj_name)
            csvwriterow.append(pouring_imagination_time)
            writer.writerow(csvwriterow)
    else:
        spill_list = [[np.nan, np.nan, np.nan], [np.nan, np.nan, np.nan],
                      [np.nan, np.nan, np.nan], [np.nan, np.nan, np.nan],
                      [np.nan, np.nan, np.nan], [np.nan, np.nan, np.nan],
                      [np.nan, np.nan, np.nan], [np.nan, np.nan, np.nan]]

    # result_txt_name = os.path.join(result_dir, obj_name + ".txt")
    # with open(result_txt_name, "w") as file1:
    #     today = time.strftime("%Y-%m-%d-%H-%M", time.localtime())
    #     file1.write("Name: " + obj_name + "\n")
    #     file1.write("Date: " + today + "\n")
    #     file1.write("Content: " + content_urdf.split('/')[-1].split('.')[0] + "\n")
    #     file1.write("Containability: " + str(containability_affordance) + "\n")
    #     file1.write("Sphere in percentage: " + str(sphere_in_percentage) + "\n")
    #     file1.write("Pour position: " + str(list(pour_pos)) + "\n")
    #     file1.write("Spill List: " + str(list(spill_list)) + "\n")
    #     file1.write("Containability imagination time: " + str(imagination_time) + "\n")
    #     file1.write("Object url: \n")