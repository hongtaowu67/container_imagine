"""
Use to sort the mAP of a method.

Author: Hongtao Wu
July 08, 2020
"""
from __future__ import division, print_function
import os
import numpy as np
from sklearn.metrics import roc_curve, auc, roc_auc_score
import matplotlib.pyplot as plt


# map_dir = "/home/hongtao/Dropbox/ICRA2021/benchmark/test_set_containability_w_everything"
# gt_dir = "/home/hongtao/Dropbox/ICRA2021/benchmark/test_set_all_gt"

map_dir = "/home/hongtao/Dropbox/ICRA2021/affnet_benchmark/affnet_map_0726/frame-000156"
gt_dir = "/home/hongtao/Dropbox/ICRA2021/affnet_benchmark/affnet_test_set_gt"

test_affnet = True

obj_list = os.listdir(gt_dir)
print ("Object number: ", len(obj_list))

dtype = [('name', np.unicode_, 50), ('map', float), ('gt', int)]
obj_info_list = []

if test_affnet:
    obj_prediction_result = dict()

for obj_filename in obj_list:
    # Read map file
    with open(os.path.join(map_dir, obj_filename), 'r') as f:
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
    
    if test_affnet:
        # Read classification file
        with open(os.path.join(map_dir, obj_name + '_classification.txt'), 'r') as f:
            line = f.readline()
            if line == 'container':
                obj_prediction_result[obj_name] = 1
            elif line == 'noncontainer':
                obj_prediction_result[obj_name] = 0
            else:
                raise ValueError(obj_name + '_classification.txt is neither container nor noncontainer!')
                
            print (obj_name, ': ', line)

    obj_info_list.append((obj_name, obj_map, obj_iscontainer))

np_obj_info_list = np.array(obj_info_list, dtype=dtype)
# print np_obj_info_list
print ("=====")
sorted_obj_info_list = np.sort(np_obj_info_list, order='map')
print (sorted_obj_info_list)

######## Calculate accuracy (imgn) ########
if not test_affnet:
    correct_classification = 0
    total_obj_num = 0
    container_num = 0

    threshold = 0.0

    for obj in obj_info_list:
        total_obj_num += 1
        obj_map = obj[1]
        if obj_map > threshold:
            obj_prediction = 1 # predict as a container
        else:
            obj_prediction = 0 # predict as a noncontainer
        
        obj_gt = obj[2]
        if obj_prediction == obj_gt:
            correct_classification += 1
        
        container_num += obj_gt

############################################

######## Calculate accuracy (affnet) ########
if test_affnet:
    correct_classification = 0
    total_obj_num = 0
    container_num = 0

    for obj in obj_info_list:
        total_obj_num += 1
        obj_prediction = obj_prediction_result[obj[0]]
        print (obj[0], ': ',obj_prediction)
        obj_gt = obj[2]
        if obj_prediction == obj_gt:
            correct_classification += 1
        container_num += obj_gt
#############################################

print ("Classification Accuracy: ", correct_classification / total_obj_num)
print ("Total num: ", total_obj_num)
print ("Container num: ", container_num)
print ("NonContainer num: ", total_obj_num - container_num)

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

# lw = 2
# plt.figure()
# plt.plot(fpr, tpr, color='darkorange',
#          lw=lw, label='ROC curve (AUC = %0.2f)' % roc_auc)
# plt.plot([0, 1], [0, 1], color='navy', lw=lw, linestyle='--')
# plt.xlim([0.0, 1.0])
# plt.ylim([0.0, 1.05])
# plt.xlabel('False Positive Rate')
# plt.ylabel('True Positive Rate')
# plt.title('Receiver Operating Characteristic curve')
# plt.legend(loc="lower right")
# plt.show()
# plt.savefig('roc_auc.png')
# plt.close()

auc = roc_auc_score(y, score)
print ("ROC AUC Score: ", auc)
###############################

    
        