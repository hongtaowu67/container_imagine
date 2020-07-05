"""
Generate ground truth labels for calculating mAP.

Author: Hongtao Wu
July 03, 2020
"""

from __future__ import division
import csv
import os

test_classification = True
check_spill = False

benchmark_dir = "/home/hongtao/Dropbox/ICRA2021/benchmark/test_set_all_gt_0703"
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
                pour_mm = int(row[-6])
                contain_mm = int(row[-5])
            
                mm_open_container = pour_mm and contain_mm

                if obj_name in obj_dict:
                    obj_dict[obj_name] += mm_open_container
                else:
                    obj_dict[obj_name] = mm_open_container
            
            else:
                raise ValueError(obj_name + " has problem with annotation.")

print "Total object number: {}".format(len(obj_dict))

for key, value in obj_dict.items():
    txt_filename = key + ".txt"

    print key, ": ", value
    
    with open(os.path.join(benchmark_dir, txt_filename), "w") as f:
        if value >= 3:
            writerow = "container 0 1 2 3"
        elif value <= 2:
            writerow = "noncontainer 0 1 2 3"

        f.write(writerow)
