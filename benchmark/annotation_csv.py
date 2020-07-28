import os
import csv
import numpy as np

'''
###### Generating new csv ######
annotation_folder = "/home/hongtao/Dropbox/ICRA2021/data/test_set_containability"
annotation_csv = "/home/hongtao/Dropbox/ICRA2021/annotation/0720/annotation_0720.csv"
obj_list = os.listdir(annotation_folder)

obj_num = len(obj_list)

obj_list_idx = np.arange(obj_num)
np.random.shuffle(obj_list_idx)

with open(annotation_csv, 'w') as file1:
    csv_writer = csv.writer(file1)
    for i in range(obj_num):
        csv_writer.writerow([obj_list[obj_list_idx[i]]])
#################################
'''

###### Add new data #####
data_folder = "/home/hongtao/Dropbox/ICRA2021/data/training_set"
annotation_csv = "/home/hongtao/Dropbox/ICRA2021/annotation/0321/annotation_0321.csv"
add_annotation_csv = "/home/hongtao/Dropbox/ICRA2021/annotation/0321/annotation_0321_train.csv"
total_obj_list = os.listdir(data_folder)
with open(annotation_csv, 'r') as f:
    csv_reader = csv.reader(f)
    annotated_obj_list = []
    for row in csv_reader:
        assert len(row) == 1
        annotated_obj_list.append(row[0])

add_obj_list = []
for obj_name in total_obj_list:
    if obj_name in annotated_obj_list:
        continue
    else:
        add_obj_list.append(obj_name)

add_obj_num = len(add_obj_list)
print "add obj num: ", add_obj_num
add_obj_list_idx = np.arange(add_obj_num)
np.random.shuffle(add_obj_list_idx)

with open(add_annotation_csv, 'w') as f:
    csv_writer = csv.writer(f)
    for i in range(add_obj_num):
        csv_writer.writerow([add_obj_list[add_obj_list_idx[i]]])
###############################


