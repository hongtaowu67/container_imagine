"""
Benchmarking the result from the containability imagination with human annotation
This script read the imagination results (txt) and calculate the container classification accuracy.

Author: Hongtao Wu
Mar 20, 2020

Add code to compute the result for all human annotation

Add code to compute the result from map result
"""

from __future__ import division

import csv
import os
import ast
import numpy as np

test_classification = True
check_spill = False
benchmark_dir = "/home/hongtao/Dropbox/ICRA2021/benchmark/test_set_all_imgn_w_tran_0703"
txt_containability_idx = 3
txt_spill_idx = 6

annotation_dir = "/home/hongtao/Dropbox/ICRA2021/annotation"
annotation = ["0312", "0321", "0409", "0417"]
annotation_all_file = "annotation_all.csv"

annotation_csv = [os.path.join(annotation_dir, annotation_group, "annotation_all", "annotation_"+ annotation_group + "_label.csv") for annotation_group  in annotation]
obj_dict = {}

for csv_path in annotation_csv:
    print "Processing " + csv_path
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
                if obj_name in obj_dict:
                    obj_dict[obj_name][0] += rice_open_container
                    obj_dict[obj_name][1] += mm_open_container
                    obj_dict[obj_name][2] += jb_open_container
                    obj_dict[obj_name][3] += db_open_container
                else:
                    obj_dict[obj_name] = open_container
            else:
                raise ValueError(obj_name + " has problem with annotation.")        

print "Total object number: {}".format(len(obj_dict))

for key, value in obj_dict.items():
    if value[0]>1 and value[0]<3:
        print key + "->rice == 2"
    if value[1]>1 and value[1]<3:
        print key + "->mm == 2"
    if value[2]>1 and value[2]<3:
        print key + "->jb == 2"
    if value[3]>1 and value[3]<3:
        print key + "->db == 2"

with open(os.path.join(annotation_dir, annotation_all_file), 'w') as file1:
    csvwriter = csv.writer(file1)
    for key, value in obj_dict.items():
        write = [key, value[0], value[1], value[2], value[3]]
        csvwriter.writerow(write)

        if value[0]>=2:
            obj_dict[key][0] = True
        else:
            obj_dict[key][0] = False

        if value[1]>=2:
            obj_dict[key][1] = True
        else:
            obj_dict[key][1] = False

        if value[2]>=2:
            obj_dict[key][2] = True
        else:
            obj_dict[key][2] = False

        if value[3]>=2:
            obj_dict[key][3] = True
        else:
            obj_dict[key][3] = False


#################### Test Classification with txt ####################
# if test_classification:

#     obj_list = os.listdir(benchmark_dir)

#     tested_obj = set()
#     correct_rice = 0
#     correct_mm = 0
#     correct_jb = 0
#     correct_db = 0
#     total_tested = 0
#     total_container_mm = 0
#     no_spill = 0


#     for obj_txt in obj_list:
#         total_tested += 1
#         obj_name = obj_txt.split('.')[0]
#         tested_obj.add(obj_name)

#         obj_path = os.path.join(benchmark_dir, obj_txt)
#         file1 = open(obj_path, 'r')

#         file_lines = file1.readlines()
#         obj_containability = file_lines[txt_containability_idx]
        
#         if "True" in obj_containability:
#             is_container = True
#         elif "False" in obj_containability:
#             is_container = False
#         else:
#             raise ValueError ("The object " + obj_name + " is not a container or a noncontainer!")

#         correct_rice += (obj_dict[obj_name][0] == is_container)
#         correct_mm += (obj_dict[obj_name][1] == is_container)
#         correct_jb += (obj_dict[obj_name][2] == is_container)
#         correct_db += (obj_dict[obj_name][3] == is_container)

#         if (obj_dict[obj_name][1] != is_container):
#             print "Containability Incorrect: " + obj_name + " " + str(obj_dict[obj_name][1])

#         if check_spill:
#             gt_obj_is_container_mm = obj_dict[obj_name][1]
#             total_container_mm += gt_obj_is_container_mm
            
#             if gt_obj_is_container_mm:    
#                 obj_spill_str = file_lines[txt_spill_idx]
                
#                 # Convert str to list
#                 if "nan" in obj_spill_str:
#                     print "Spill object: {} is not identified as an open container".format(obj_name)   
#                 else:
#                     obj_spill_list = ast.literal_eval(obj_spill_str.split(':')[1].split('\n')[0][1:])
#                     obj_spill_list = np.array(obj_spill_list).flatten()

#                     obj_no_spill = False
#                     for spill in obj_spill_list:
#                         if spill < 7:
#                             obj_no_spill = True
#                             break

#                     no_spill += obj_no_spill

#                     if not obj_no_spill:
#                         print "Spill object: cannot pour into {} ".format(obj_name)    

#     if total_tested != len(tested_obj):
#         raise ValueError("Some objects have been tested more than once!")
#     print "Total tested object number: {}".format(len(tested_obj))
#     print "Contain Rice Accuracy: {}".format(correct_rice / len(tested_obj))
#     print "Contain M&M Accuracy: {}".format(correct_mm / len(tested_obj))
#     print "Contain Jelly Beans Accuracy: {}".format(correct_jb / len(tested_obj))
#     print "Contain Decorative Beads Accuracy: {}".format(correct_db / len(tested_obj))

#     if check_spill:
#         print "No Spillage Rate: {}".format(no_spill / total_container_mm)
#################### Test Classification with txt ####################

#################### Test Classification with mAP ####################
obj_list = os.listdir(benchmark_dir)

tested_obj = set()

correct_mm = 0
total_tested = 0

for obj_txt in obj_list:
    total_tested += 1
    obj_name = obj_txt.split('.')[0]
    # print "obj_name: ", obj_name
    tested_obj.add(obj_name)

    obj_path = os.path.join(benchmark_dir, obj_txt)
    file1 = open(obj_path, 'r')

    file_lines = file1.readlines()
    obj_containability = float(file_lines[0].split(" ")[1])
    # print "obj_containability: ", obj_containability
    
    if obj_containability > 0.0:
        is_container = True
    else:
        is_container = False

    correct_mm += (obj_dict[obj_name][1] == is_container)


    if (obj_dict[obj_name][1] != is_container):
        print "Containability Incorrect: " + obj_name + " " + str(obj_dict[obj_name][1])

    
if total_tested != len(tested_obj):
    raise ValueError("Some objects have been tested more than once!")
print "Total tested object number: {}".format(len(tested_obj))
print "Contain M&M Accuracy: {}".format(correct_mm / len(tested_obj))

       
#################### Test Classification with mAP ####################


