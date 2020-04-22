"""
Benchmarking the result from the containability imagination with human annotation
This script read the imagination results (txt) and calculate the container classification accuracy.

Author: Hongtao Wu
Mar 20, 2020
"""

from __future__ import division

import csv
import os
import ast
import numpy as np

benchmark_dir = "/home/hongtao/Dropbox/ICRA2021/benchmark/0406_mm"
txt_containability_idx = 3
txt_spill_idx = 6
check_spill = True

csv_path = "/home/hongtao/Dropbox/ICRA2021/annotation/0321/annotation_all/annotation_0321_label.csv"
obj_dict = {}

with open(csv_path, 'r') as csvfile:
    csv_reader = csv.reader(csvfile, delimiter=',')
    for row in csv_reader:
        obj_name = row[0]
        obj_name_check = row[1]
        if obj_name == obj_name_check:
            pour_rice = int(row[-8])
            contain_rice = int(row[-7])
            pour_mm = int(row[-6])
            contain_mm = int(row[-5])
            pour_jb = int(row[-4])
            contain_jb = int(row[-3])
            pour_db = int(row[-2])
            contain_db = int(row[-1])

            rice_open_container = pour_rice and contain_rice
            mm_open_container = pour_mm and contain_mm
            jb_open_container = pour_jb and contain_jb
            db_open_container = pour_db and contain_db
            open_container = [rice_open_container, mm_open_container, jb_open_container, db_open_container]
            obj_dict[obj_name_check] = open_container
        else:
            raise ValueError(obj_name + "has problem with annotation.")        

print "Total object number: {}".format(len(obj_dict))

obj_list = os.listdir(benchmark_dir)

tested_obj = set()
correct_rice = 0
correct_mm = 0
correct_jb = 0
correct_db = 0
total_tested = 0
total_container_mm = 0
no_spill = 0


for obj_txt in obj_list:
    total_tested += 1
    obj_name = obj_txt.split('.')[0]
    tested_obj.add(obj_name)

    obj_path = os.path.join(benchmark_dir, obj_txt)
    file1 = open(obj_path, 'r')

    file_lines = file1.readlines()
    obj_containability = file_lines[txt_containability_idx]
    
    if "True" in obj_containability:
        is_container = 1
    elif "False" in obj_containability:
        is_container = 0
    else:
        raise ValueError ("The object " + obj_name + " is not a container or a noncontainer!")

    correct_rice += (obj_dict[obj_name][0] == is_container)
    correct_mm += (obj_dict[obj_name][1] == is_container)
    correct_jb += (obj_dict[obj_name][2] == is_container)
    correct_db += (obj_dict[obj_name][3] == is_container)

    if (obj_dict[obj_name][1] != is_container):
        print "Containability Incorrect: " + obj_name

    if check_spill:
        gt_obj_is_container_mm = obj_dict[obj_name][1]
        total_container_mm += gt_obj_is_container_mm
        
        if gt_obj_is_container_mm:    
            obj_spill_str = file_lines[txt_spill_idx]
            
            # Convert str to list
            if "nan" in obj_spill_str:
                print "Spill object: {} is not identified as an open container".format(obj_name)   
            else:
                obj_spill_list = ast.literal_eval(obj_spill_str.split(':')[1].split('\n')[0][1:])
                obj_spill_list = np.array(obj_spill_list).flatten()

                obj_no_spill = False
                for spill in obj_spill_list:
                    if spill < 2:
                        obj_no_spill = True
                        break

                no_spill += obj_no_spill

                if not obj_no_spill:
                    print "Spill object: cannot pour into {} ".format(obj_name)            


if total_tested != len(tested_obj):
    raise ValueError("Some objects have been tested more than once!")
print "Total tested object number: {}".format(len(tested_obj))
print "Contain Rice Accuracy: {}".format(correct_rice / len(tested_obj))
print "Contain M&M Accuracy: {}".format(correct_mm / len(tested_obj))
print "Contain Jelly Beans Accuracy: {}".format(correct_jb / len(tested_obj))
print "Contain Decorative Beads Accuracy: {}".format(correct_db / len(tested_obj))

if check_spill:
    print "No Spillage Rate: {}".format(no_spill / total_container_mm)