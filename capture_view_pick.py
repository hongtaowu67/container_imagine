#!/usr/bin/env python
"""
Automatically capture frames to reconstruct the object. The gripper is mounted on the robot.
@author: Hongtao Wu
Jan 13, 2020
"""

import os
import time

import numpy as np
import cv2
import rospy

from robot import Robot
from reconstruction.ros_camera_tsdf_fusion import ROSCameraTSDFFusion


class AutoCapture:
    def __init__ (self, data_folder, acc, vel, cam2ee_file, automatic=True, time_interval=None):
        self.automatic = automatic
        if not self.automatic:
            self.time_interval = time_interval # time interval between each capture

        self.data_folder = data_folder # data folder for saving the data
        self.cam = ROSCameraTSDFFusion(automatic=self.automatic)
        rospy.init_node('ros_camera_tsdf_fusion', anonymous=False)

        self.acc = acc
        self.vel = vel

        self.robot = Robot(acc=self.acc, vel=self.vel, gripper_on=False)

        # camera in ee frame
        with open(os.path.join(cam2ee_file), 'r') as file_robot:
            cam2ee_str = file_robot.readline().split(' ')
            cam2ee = [float (x) for x in cam2ee_str if x is not '']
            assert len(cam2ee) == 16
            self.cam2ee = np.reshape(np.array(cam2ee), (4, 4))

        # Robot joint for each view (24 views)
        # Right Robot
        self.rob_joints_view = [
            # Front to back
            (1.3270788192749023, -0.5871318022357386, 0.5635156631469727, -0.9754813353167933, -1.7128685156451624, -0.9983609358416956),
            (1.326743245124817, -0.7666080633746546, 0.5507659912109375, -1.1000788847552698, -1.6169183889972132, -0.8689840475665491),
            (1.3360190391540527, -0.9469612280475062, 0.5468959808349609, -0.9377940336810511, -1.6170862356769007, -0.947662655507223),
            (1.3359711170196533, -1.1091974417315882, 0.543241024017334, -1.1184561888324183, -1.6170504728900355, -0.9476144949542444),
            (1.3097859621047974, -1.325947109852926, 0.5417914390563965, -1.2099135557757776, -1.6170862356769007, -0.9475906530963343),
            (1.3063464164733887, -1.531799618397848, 0.5402097702026367, -1.2114842573748987, -1.6275361219989222, -0.9428218046771448),
            (1.3062745332717896, -1.8865845839129847, 0.5398621559143066, -1.3007505575763147, -1.6275718847857874, -0.9428337256060999),
            (1.306550145149231, -2.303037468587057, 0.5397424697875977, -1.3006184736834925, -1.7889497915851038, -0.941359821950094),
            (1.3082759380340576, -2.663814369832174, 0.5396947860717773, -1.2180894056903284, -1.7824023405658167, -0.9413002173053187),
            (1.3044769763946533, -2.9997742811786097, 0.5389156341552734, -1.218149487172262, -1.5771907011615198, -0.7237361113177698),            
            
            # Right
            (0.1635962873697281, -3.0001819769488733, 0.5388917922973633, -1.2183292547809046, -1.2167356649981897, -0.7237480322467249),
            (-0.11921912828554326, -3.0355380217181605, 0.3965644836425781, -0.9881036917315882, -1.2167237440692347, -0.7238200346576136),
            (-0.21987420717348272, -2.6819637457477015, 0.40045928955078125, -0.9391124884234827, -1.2167118231402796, -0.7238319555865687),
            (-0.1557830015765589, -2.3539393583880823, 0.3219118118286133, -0.8757150808917444, -1.3472612539874476, -0.8076909224139612),

            # Left
            (3.273634672164917, -3.1482985655414026, 0.3076972961425781, -0.8725264708148401, -2.063065830861227, -0.9009659926043909),
            (2.869597911834717, -2.959958855305807, 0.3995485305786133, -0.8721903006183069, -1.982847038899557, -0.900977913533346),
            (3.44594407081604, -2.7235000769244593, 0.32242679595947266, -0.8720467726336878, -2.076796833668844, -0.9009659926043909),
            (3.5350141525268555, -2.4529956022845667, 0.32257509231567383, -0.8774879614459437, -2.1872618834124964, -0.8489778677569788),

            # Front to back
            # (1.3270788192749023, -0.5871318022357386, 0.5635156631469727, -0.9754813353167933, -1.7128685156451624, -0.9983609358416956),
            (1.3360190391540527, -0.9469612280475062, 0.5468959808349609, -0.9377940336810511, -1.6170862356769007, -0.947662655507223),
            (1.3097859621047974, -1.325947109852926, 0.5417914390563965, -1.2099135557757776, -1.6170862356769007, -0.9475906530963343),
            (0.8367955088615417, -1.8993938604937952, 1.2887015342712402, -1.6408045927630823, -1.0915311018573206, -1.3117168585406702),
            # (1.3082759380340576, -2.663814369832174, 0.5396947860717773, -1.2180894056903284, -1.7824023405658167, -0.9413002173053187),
            (1.3044769763946533, -2.9997742811786097, 0.5389156341552734, -1.218149487172262, -1.5771907011615198, -0.7237361113177698),

            # Right
            # (-0.7737506071673792, -3.105339829121725, 0.22678089141845703, -0.6576903502093714, -1.2063811461078089, -0.7237842718707483),
            (-0.0524066130267542, -3.0054448286639612, 0.3074216842651367, -0.605147663746969, -1.288990322743551, -0.7238920370685022),
            # (0.1567772626876831, -3.0374682585345667, 0.21999645233154297, -0.9742463270770472, -1.2818196455584925, -0.7237961927997034),

            # Left
            (2.389786720275879, -3.1370280424701136, 0.3974032402038574, -0.7821973005877894, -1.9841755072223108, -0.9009302298175257),
            # (1.8424330949783325, -3.136968437825338, 0.39767932891845703, -0.7822216192828577, -1.9841278235064905, -0.9009059111224573),
            # (3.2762231826782227, -3.1569674650775355, 0.39711570739746094, -0.7731722036944788, -2.1216662565814417, -0.9009421507464808)         
            
        ]

        self.pick_bottle_joints = [
            # Low
            (2.006856679916382, -1.3167336622821253, 1.6932334899902344, -1.9662039915667933, -1.5629408995257776, -1.0797608534442347),
            # High
            (2.007300853729248, -1.3760116736041468, 1.5031867027282715, -1.7167752424823206, -1.5620291868792933, -1.0819900671588343)
        ]

        
    def save_frame(self, idx):
        print('Start Capturing the frame...')
        
        rgb_file_name = 'frame-{:06d}.color.png'.format(idx+150)
        depth_file_name = 'frame-{:06d}.depth.png'.format(idx+150)
        pose_file_name = 'frame-{:06d}.pose.txt'.format(idx+150)

        if not self.automatic:
            rgb_img, depth_img, camera_pose = self.cam.get_frame()
        else:
            rgb_img, depth_img, _ = self.cam.get_frame()
            robot_pose = self.robot.get_pose()
            camera_pose = np.matmul(robot_pose, self.cam2ee)

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

            print('Finish number {} frame!'.format(idx))
            save_success = True

        else:
            print('Nothing is saved! There is missing data from rgb_img, depth_img, or camera_pose!')
            print('Please check!')

    
    def collect_data(self):
        for idx, rob_joint in enumerate(self.rob_joints_view):
            self.robot.move_to_joint(rob_joint)
            self.save_frame(idx)

        self.robot.go_home()
        self.robot.disconnect()
        


# Test
if __name__ == "__main__":
    root_folder = os.getcwd()
    
    data_name = "GripperTest_24view"

    data_folder = os.path.join("/home/hongtao/Dropbox/ICRA2021/data", data_name)
    if not os.path.exists(data_folder):
        os.mkdir(data_folder)
        os.mkdir(os.path.join(data_folder, 'rgbd'))
    cam2ee_file = os.path.join(root_folder, "calibrate/camera_pose.txt")
    ACT = AutoCapture(data_folder=os.path.join(data_folder, 'rgbd'), 
                                acc=1.0, vel=1.0, cam2ee_file=cam2ee_file)
    ACT.collect_data()
