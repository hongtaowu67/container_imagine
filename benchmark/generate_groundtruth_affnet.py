"""
Generate ground truth for the small dataset tested on affnet.
If the object belongs to cup, pan, or bowl class, it is classified as open container by default;
else noncontainer.

But ultimately, this code will be  deprecate.
Human annotation will take over.

Author: Hongtao Wu
July 5, 2020
"""

import os

map_dir = "/home/hongtao/Dropbox/ICRA2021/affnet_benchmark/gt_map"
data_dir = "/home/hongtao/Dropbox/ICRA2021/affnet_benchmark/affnet_benchmark_crop"
class_folders = os.listdir(data_dir)

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
            else:
                writerow = "noncontainer 0 1 2 3"
            f.write(writerow)
