# Benchmark containability imagination result with classification accuracy and mAP

# python benchmark_map.py <result_dir> <gt_dir>

# Author: Hongtao Wu
# Institution: Johns Hopkins University
# Date: July 08, 2020

from __future__ import division, print_function
import os
import argparse
import numpy as np
from sklearn.metrics import roc_curve, auc, roc_auc_score
import matplotlib.pyplot as plt

parser = argparse.ArgumentParser(
    description="Containability imagination result accuracy and mAP benchmark")
parser.add_argument(
    "result_dir",
    type=str,
    help="directory of the result of containability imagination")
parser.add_argument("gt_dir", type=str, help="directory of the ground truth")

args = parser.parse_args()

result_dir = args.result_dir
gt_dir = args.gt_dir

# If the sphere-in-percentage is larger than 0.0
# the object is classified as an open container
classification_threshold = 0.0

obj_list = os.listdir(gt_dir)
print("Object number: ", len(obj_list))

dtype = [('name', np.unicode_, 50), ('map', float), ('gt', int)]
obj_info_list = []

for obj_filename in obj_list:
    # Read map file
    with open(os.path.join(result_dir, obj_filename), 'r') as f:
        line = f.readline()
        items = line.split(' ')
        obj_name = obj_filename.split('.')[0]
        obj_map = float(items[1])

    # Read gt file
    with open(os.path.join(gt_dir, obj_filename), 'r') as f:
        line = f.readline()
        container_str = line.split(' ')[0]
        if container_str == 'container':
            obj_iscontainer = 1
        else:
            obj_iscontainer = 0

    obj_info_list.append((obj_name, obj_map, obj_iscontainer))

np_obj_info_list = np.array(obj_info_list, dtype=dtype)
print("=====")
sorted_obj_info_list = np.sort(np_obj_info_list, order='map')
print(sorted_obj_info_list)

######## Calculate accuracy (imgn) ########
correct_classification = 0
total_obj_num = 0
container_num = 0

for obj in obj_info_list:
    total_obj_num += 1
    obj_map = obj[1]
    if obj_map > classification_threshold:
        obj_prediction = 1  # predict as a container
    else:
        obj_prediction = 0  # predict as a noncontainer

    obj_gt = obj[2]
    if obj_prediction == obj_gt:
        correct_classification += 1

    container_num += obj_gt

############################################

print("Classification Accuracy: ", correct_classification / total_obj_num)
print("Total num: ", total_obj_num)
print("Container num: ", container_num)
print("NonContainer num: ", total_obj_num - container_num)

######### Calculate AUC #######
y = []
score = []

for obj in obj_info_list:
    obj_iscontainer_gt = obj[2]
    obj_iscontainer_pred = obj[1]
    y.append(obj_iscontainer_gt)
    score.append(obj_iscontainer_pred)
fpr, tpr, _ = roc_curve(y, score, pos_label=1)
roc_auc = auc(fpr, tpr)

auc = roc_auc_score(y, score)
print("ROC AUC Score: ", auc)
###############################
