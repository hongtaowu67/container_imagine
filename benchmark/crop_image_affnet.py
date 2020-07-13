"""
Crop the images based on the annotation json for benchmarking with AffordanceNet.

Author: Hongtao Wu
July 03, 2020
"""

import cv2
import json
import os

output_folder = "/home/hongtao/Dropbox/ICRA2021/affnet_benchmark/affnet_benchmark_crop"
data_folder = "/home/hongtao/Dropbox/ICRA2021/affnet_benchmark/affnet_benchmark_object"
total_img_num = 24

class_list = ["spatula"]

for class_name in class_list:
    obj_list = ["Rubbermaid_spatula"]
    # obj_list = os.listdir(os.path.join(data_folder, class_name))
    
    for obj_name in obj_list:
        
        crop_img_dir = os.path.join(output_folder, obj_name)
        if os.path.exists(crop_img_dir):
            pass
        else:
            os.mkdir(crop_img_dir)

        rgbd_dir = os.path.join(data_folder, class_name, obj_name, "rgbd")
        json_filename = obj_name + "_bbox.json"
        json_path = os.path.join(data_folder, class_name, obj_name, json_filename)
        print "json path: ", json_path

        with open(json_path) as f:
            bbox_dict = json.load(f)
            img_num = 0
            for (key, value) in bbox_dict.items():
                img_num += 1
                img_name = value["filename"]
                img = cv2.imread(os.path.join(rgbd_dir, img_name))

                print img_name
                
                img_h = img.shape[0]
                img_w = img.shape[1]
                print "img_h, img_w: ", img_h, img_w

                x = value["regions"][0]["shape_attributes"]["x"]
                y = value["regions"][0]["shape_attributes"]["y"]
                width = value["regions"][0]["shape_attributes"]["width"]
                height = value["regions"][0]["shape_attributes"]["height"]

                print "x, y: ", x, y
                print "width, height: ", width, height
                print "======="

                x_1 = max(0, x)
                y_1 = max(0, y)

                x_2 = min(x+width, img_w)
                y_2 = min(y+height, img_h)


                crop_img = img[y_1:y_2, x_1:x_2]
                crop_img_name = img_name.split(".")[0] + ".crop.png"
                crop_img_path = os.path.join(crop_img_dir, crop_img_name)
                cv2.imwrite(crop_img_path, crop_img)
        
        assert img_num == total_img_num
