"""
Benchmarking the result from the containability imagination.
This script read the imagination results (txt) and calculate the container classification accuracy.

Author: Hongtao Wu
Mar 10, 2020
"""

import os

data_folder = "/home/hongtao/Dropbox/ICRA2021/data/test_set/NonContainer"
# the index for the containability in the txt file
txt_containability_idx = 2
obj_list = os.listdir(data_folder)

obj_num = 0
container_num = 0
noncontainer_num = 0
for obj_name in obj_list:
    obj_num += 1
    obj_txt = obj_name + ".txt"
    obj_path = os.path.join(data_folder, obj_name, obj_txt)
    file1 = open(obj_path, 'r')

    obj_containability = file1.readlines()[txt_containability_idx]
    if "True" in obj_containability:
        container_num += 1
        print("Container: ", obj_name)
    elif "False" in obj_containability:
        noncontainer_num += 1
        # print("Noncontainer: ", obj_name)
    else:
        raise ValueError ("The object " + obj_name + " is not a container or a noncontainer!")


print("Total Object Num: {}".format(obj_num))
print("Container Num: {}".format(container_num))
print("Non-container Num: {}".format(noncontainer_num))


