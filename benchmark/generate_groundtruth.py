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

benchmark_dir = "/home/hongtao/Dropbox/ICRA2021/benchmark/training_set_containability_gt"
annotation_dir = "/home/hongtao/Dropbox/ICRA2021/annotation"
annotation = ["0312", "0321", "0409", "0417", "0720"]

# Test set annotation
# annotation_csv = [os.path.join(annotation_dir, annotation_group, "annotation_all", "annotation_"+ annotation_group + "_label.csv") for annotation_group  in annotation]

# Training set annotation
annotation_csv = []
for annotation_group in annotation:
    annotation_group_dir = os.path.join(annotation_dir, annotation_group, "annotation_all")
    filenames = os.listdir(annotation_group_dir)
    for filename in filenames:
        if "_train_label" in filename:
            annotation_csv.append(os.path.join(annotation_group_dir, filename))

obj_dict = {}
obj_annotation_count = {}
    
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

                assert pour_mm is not None
                assert contain_mm is not None
            
                mm_open_container = pour_mm and contain_mm

                if obj_name in obj_dict:
                    obj_dict[obj_name] += mm_open_container
                    obj_annotation_count[obj_name] += 1
                else:
                    obj_dict[obj_name] = mm_open_container
                    obj_annotation_count[obj_name] = 1
            
            else:
                raise ValueError(obj_name + " has problem with annotation.")

print "Total object number: {}".format(len(obj_dict))

num_container = 0
num_noncontainer = 0


############## Write Groundtruth ################
for key, value in obj_dict.items():
    txt_filename = key + ".txt"
    print key, ": ", value, ", ", obj_annotation_count[key]
    assert obj_annotation_count[key] == 5

    
    with open(os.path.join(benchmark_dir, txt_filename), "w") as f:
        if value >= 3:
            writerow = "container 0 1 2 3"
            num_container += 1

        else:
            writerow = "noncontainer 0 1 2 3"
            num_noncontainer += 1
        
        print key, ": ", value, " container/ ", (5-value), " noncontainer"

        f.write(writerow)
    print "====="

print "num_container: ", num_container
print "num_noncontainer: ", num_noncontainer
################################################


# data_dir = "/home/hongtao/Dropbox/ICRA2021/affnet_benchmark/affnet_benchmark_object"
# class_folders = os.listdir(data_dir)
# obj_list = []

# for class_name in class_folders:
#     class_dir = os.path.join(data_dir, class_name)
#     obj_folders = os.listdir(class_dir)

#     for obj in obj_folders:
#         obj_list.append(obj)

# print "Number of objects: ", len(obj_list)

# for key, value in obj_dict.items():
#     if key in obj_list:
#         print key, ": ", value