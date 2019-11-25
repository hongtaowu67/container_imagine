"""
Capture rgb, depth, and compute camera pose for 3D reconstruction of the scene.
An ArUco tag and an objet on a table. 
Color frame for localizing the camera. Depth frame for reconstruction.
@author: Hongtao Wu
Nov 23, 2019
"""

import roslib
import rospy

import numpy as np
import cv2
import os
import time

from ros_camera_tsdf_fusion import ROSCameraTSDFFusion

class CaptureTSDFFusion:
    def __init__ (self, time_interval, data_folder):
        self.time_interval = time_interval # time interval between each capture
        self.data_folder = data_folder # data folder for saving the data
        self.cam = ROSCameraTSDFFusion()
        rospy.init_node('ros_camera_tsdf_fusion', anonymous=False)

    def save_frame(self, idx):
#        print('Start Capturing the frame...')
#        print('Please move the camera to a fixed position.')
#        print('The time for adjusting the camera is %d seconds.' % (self.time_interval))
        time.sleep(self.time_interval)
        
        rgb_file_name = 'frame-{:06d}.color.png'.format(i)
        depth_file_name = 'frame-{:06d}.depth.png'.format(i)
        pose_file_name = 'frame-{:06d}.pose.txt'.format(i)
        pose_inverse_file_name = 'frame-{:06}.inversepose.txt'.format(i)

        rgb_img, depth_img, camera_pose = self.cam.get_frame()

        save_success = False

        if rgb_img is not None:
            # Save rgb image
            cv2.imwrite(os.path.join(self.data_folder, rgb_file_name), rgb_img)
            # Save depth image
            cv2.imwrite(os.path.join(self.data_folder, depth_file_name), depth_img)

            # Save pose txt file
            f = open(os.path.join(self.data_folder, pose_file_name), 'w')
            for line in camera_pose:
                writeLine = str(line[0]) + ' ' + str(line[1]) + ' ' + str(line[2]) + ' ' + str(line[3]) + '\n'
                f.write(writeLine)
            
            f.close()

            # Save inverse pose txt file
            inverse_camera_pose = np.linalg.inv(camera_pose)
            f = open(os.path.join(self.data_folder, pose_inverse_file_name), 'w')
            for line in inverse_camera_pose:
                writeLine = str(line[0]) + ' ' + str(line[1]) + ' ' + str(line[2]) + ' ' + str(line[3]) + '\n'
                f.write(writeLine)
            
            f.close()

            print('Finish number {} frame!'.format(idx))
            save_success = True
        
        else:
            print('Nothing is saved! There is missing data from rgb_img, depth_img, or camera_pose!')
            print('Please check!')
        
        return save_success


if __name__ == '__main__':
    time_interval = 3
    data_folder = '/home/hongtao/src/cup_imagine/reconstruction/data'
    Capture = CaptureTSDFFusion(time_interval, data_folder)
    
    frame_num = 50
    for i in range(frame_num):
        flag = False
        while not flag:
            flag = Capture.save_frame(i)
    
    print('Finish {} frames'.format(frame_num))
