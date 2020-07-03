import os
import csv
import numpy as np

annotation_folder = "/home/hongtao/Dropbox/ICRA2021/data/test_set_all"
annotation_csv = "/home/hongtao/Dropbox/ICRA2021/annotation/0417/annotation_0417.csv"
obj_list = os.listdir(annotation_folder)

obj_num = len(obj_list)

obj_list_idx = np.arange(obj_num)
np.random.shuffle(obj_list_idx)

with open(annotation_csv, 'w') as file1:
    csv_writer = csv.writer(file1)
    writelines = []
    for i in range(obj_num):
        csv_writer.writerow([obj_list[obj_list_idx[i]]])


