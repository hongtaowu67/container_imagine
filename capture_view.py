"""
Automatically capture frames to reconstruct the object.
"""

import os
import time

import numpy as np
import cv2
import rospy

from robot import Robot
from reconstruction.ros_camera_tsdf_fusion import ROSCameraTSDFFusion



class AutoCaptureTSDFFusion:
    def __init__ (self, data_folder, acc, vel, cam2ee_file, automatic=True, time_interval=None):
        self.automatic = automatic
        if not self.automatic:
            self.time_interval = time_interval # time interval between each capture

        self.data_folder = data_folder # data folder for saving the data
        self.cam = ROSCameraTSDFFusion(automatic=self.automatic)
        rospy.init_node('ros_camera_tsdf_fusion', anonymous=False)

        self.acc = acc
        self.vel = vel

        self.robot = Robot(acc=self.acc, vel=self.vel)

        # camera in ee frame
        with open(os.path.join(cam2ee_file), 'r') as file_robot:
            cam2ee_str = file_robot.readline().split(' ')
            cam2ee = [float (x) for x in cam2ee_str if x is not '']
            assert len(cam2ee) == 16
            self.cam2ee = np.reshape(np.array(cam2ee), (4, 4))

        # Robot joint for each view
        self.rob_joints_view = [
                  (1.3270788192749023, -0.5871318022357386, 0.5635156631469727, -0.9754813353167933, -1.7128685156451624, -0.9983609358416956),
                  (1.6738520860671997, -0.909912411366598, 1.198838233947754, -1.3837016264544886, -1.8483217398272913, -1.2562173048602503),
                  (2.275864601135254, -1.1361377874957483, 1.6057133674621582, -1.5455578009234827, -2.0074594656573694, -1.4950335661517542),
                  (2.9399449825286865, -1.8619125525103968, 2.4148547649383545, -1.7534025351153772, -2.121786419545309, -1.7017815748797815),
                  (0.8367955088615417, -1.8993938604937952, 1.2887015342712402, -1.6408045927630823, -1.0915311018573206, -1.3117168585406702),

                  (-0.06937629381288701, -1.538771931325094, 2.1277785301208496, -1.6894596258746546, -1.2202795187579554, -0.13213998476137334),
                  (0.4341154396533966, -0.9359772841082972, 1.2647509574890137, -1.3903196493731897, -1.3129308859454554, -0.46772605577577764),
                  (0.8396477699279785, -0.6254847685443323, 0.7492246627807617, -1.1100171248065394, -1.4959281126605433, -0.6273883024798792),
                    
                  (1.3122786283493042, -0.8231795469867151, 0.7763476371765137, -1.175483528767721, -1.685948673878805, -1.0210440794574183),
                  (1.164118766784668, -1.3849709669696253, 0.853243350982666, -1.510956112538473, -1.3925560156451624, -1.1384103933917444),
                  (0.8367955088615417, -1.8993938604937952, 1.2887015342712402, -1.6408045927630823, -1.0915311018573206, -1.3117168585406702),
                  (4.33081579208374, -0.8743260542498987, -1.5989578405963343, -1.0231664816485804, 1.9429339170455933, -0.9170468489276331),
                  (4.7088727951049805, 0.3207979202270508, -2.761106077824728, -0.40907174745668584, 1.597633957862854, -0.7615912596331995),

                  (3.327347755432129, -1.370307747517721, -2.151564900075094, -0.49612361589540654, 2.728957176208496, -0.6719558874713343),
                  (3.903167724609375, -1.8390501181231897, -1.7808659712420862, -0.9992039839373987, 2.6271567344665527, -0.9850242773639124),
                  (4.061360836029053, -1.3026106993304651, -1.75623065630068, -0.9523699919330042, 2.183171510696411, -0.7390387693988245),
                  (4.169902324676514, -2.223264996205465, -0.840802017842428, -1.6988595167743128, 2.3230490684509277, -0.8963406721698206),

                  (1.3565360307693481, -2.9083827177630823, 2.2230334281921387, -2.0387242476092737, -1.1163714567767542, -1.764909569417135),
                  (2.0507302284240723, -1.5372627417193812, 1.6491880416870117, -1.5733378569232386, -1.7753518263446253, -1.8432648817645472),
                  (1.714035153388977, -1.128387753163473, 1.1511139869689941, -1.4616559187518519, -1.6742060820208948, -1.4307540098773401),
                  (0.8367955088615417, -1.8993938604937952, 1.2887015342712402, -1.6408045927630823, -1.0915311018573206, -1.3117168585406702)
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


# Test
if __name__ == "__main__":
    root_folder = os.getcwd()
    data_name = "12-23-19"
    data_folder = os.path.join(root_folder, "data", data_name)
    cam2ee_file = os.path.join(root_folder, "calibrate/camera_pose.txt")
    ACT = AutoCaptureTSDFFusion(data_folder=data_folder, acc=0.2, vel=0.2, cam2ee_file=cam2ee_file)
    ACT.collect_data()
