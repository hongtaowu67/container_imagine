"""
This script is written for annotation of containability and pouring availability.
The annotation will be saved in a csv file.

Author: Hongtao Wu
Mar 09, 2020
"""

import csv
import os

def main(ans1, ans2, ans3, ans4, ans5, ans6):
    # Annotator's name
    annotator_name = "Sipu_Ruan"

    # Object info
    dataset = "/home/hongtao/Dropbox/ICRA2021/data/test_set"
    obj_folder = "container"
    obj_name = "Amazon_Accessory_Tray"
    obj_path = os.path.join(dataset, obj_folder, obj_name)

    # Annotation info
    annotation_folder = "/home/hongtao/Dropbox/ICRA2021/data/annotation"
    annotation_file = obj_name + "_" + annotator_name + ".csv"
    csv_path = os.path.join(annotation_folder, annotation_file)

    if os.path.exists(csv_path):
        raise ValueError(annotation_file + " already exists in " + annotation_folder + "!")

    csvwriteline = [annotator_name]
    csvwriteline.append(obj_name)
    csvwriteline.append(ans1)
    csvwriteline.append(ans2)
    csvwriteline.append(ans3)
    csvwriteline.append(ans4)
    csvwriteline.append(ans5)
    csvwriteline.append(ans6)

    # Save the annotation file
    with open(csv_path, 'w') as file1:
        csv_writer = csv.writer(file1)
        csv_writer.writerow(csvwriteline)

if __name__ == "__main__":
    
    # Annotation
    # Question 1: Given this object in this pose, are you able to pour into it?
    # Question 2: Given the object in this pose, do you think it can serve as a container 
    #             to contain decorative beads ()?
    # Question 3: Given the object in this pose, do you think it can serve as a container 
    #             to contain m&ms ()?
    # Question 4: Given the object in this pose, do you think it can serve as a container
    #             to contain jelly beans ()?  
    # Question 5: Given the object in this pose, do you think it can serve as a container 
    #             to contain ping-pong ball ()?
    # Question 6: What is the volume of this container? ( [0, 100); [100, 500); [500, 1000); [1000, inf) )
    
    ans1 = 1 # 1: True, 0: False
    ans2 = 1 # 1: True, 0: False
    ans3 = 1 # 1: True, 0: False
    ans4 = 1 # 1: True, 0: False
    ans5 = 1 # 1: True, 0: False
    ans6 = 0 # the smaller limit of the range, e.g., 0 for [0, 100)

    main(ans1, ans2, ans3, ans4, ans5, ans6)
    
