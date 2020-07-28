"""
Generate ground truth for the small dataset tested on affnet.
If the object belongs to cup, pan, or bowl class, it is classified as open container by default;
else noncontainer.

But ultimately, this code will be  deprecate.
Human annotation will take over.

Author: Hongtao Wu
July 5, 2020

Change to extract gt from the test_set_containability_gt folder which contains all the gt files
Can also used to move map result from test_set_containability
"""

import os
'''
map_dir = "/home/hongtao/Dropbox/ICRA2021/benchmark/test_set_all_gt"
data_dir = "/home/hongtao/Dropbox/ICRA2021/affnet_benchmark/affnet_benchmark_object"
class_folders = os.listdir(data_dir)

num_container = 0
num_noncontainer = 0

for class_name in class_folders:
    if class_name == "bowl" or class_name == "cup" or class_name == "pan":
        obj_iscontainer = True
    else:
        obj_iscontainer = False
    
    class_dir = os.path.join(data_dir, class_name)
    object_folders = os.listdir(class_dir)

    for object_name in object_folders:
        map_filename = object_name + ".txt"
        map_path = os.path.join(map_dir, map_filename)

        with open(map_path, 'w') as f:
            if obj_iscontainer:
                writerow = "container 0 1 2 3"
                num_container += 1
            else:
                writerow = "noncontainer 0 1 2 3"
                num_noncontainer += 1
            f.write(writerow)

print "container num: ", num_container
print "noncontainer num: ", num_noncontainer
'''

import shutil
gt_source_dir = "/home/hongtao/Dropbox/ICRA2021/benchmark/test_set_containability_w_everything_0726_mp4"
affnet_gt_target_dir = "/home/hongtao/Dropbox/ICRA2021/affnet_benchmark/imgn_map_0726_mp4"

data_dir = "/home/hongtao/Dropbox/ICRA2021/affnet_benchmark/affnet_benchmark_object"
class_folders = os.listdir(data_dir)
num_container = 0
num_noncontainer = 0

for class_name in class_folders:
    class_dir = os.path.join(data_dir, class_name)
    object_folders = os.listdir(class_dir)

    for object_name in object_folders:
        # gt_filename = object_name + ".txt"
        gt_filename = object_name + "_contain.mp4"
        gt_path = os.path.join(gt_source_dir, gt_filename)
        affnet_gt_path = os.path.join(affnet_gt_target_dir, gt_filename)
        shutil.copy(gt_path, affnet_gt_path)