#! /usr/bin/env python

"""
Calibration with an ArUco tag.
Use aruco_ros ROS package to detect the pose of the tag.
Author: Hongtao Wu
"""


import numpy as np
import rospy
import time
import cv2
import os

from robot import Robot
from utils import make_rigid_transformation, quat2rotm, pose_inv, get_mat_log
from calibrate.aruco import ArUco


def quat2rotm(quat):
    """
    Quaternion to rotation matrix.
    
    Args:
    - quat (4, numpy array): quaternion w, x, y, z

    Returns:
    - rotm: (3x3 numpy array): rotation matrix
    """
    w = quat[0]
    x = quat[1]
    y = quat[2]
    z = quat[3]

    s = w*w + x*x + y*y + z*z

    rotm = np.array([[1-2*(y*y+z*z)/s, 2*(x*y-z*w)/s,   2*(x*z+y*w)/s  ],
                     [2*(x*y+z*w)/s,   1-2*(x*x+z*z)/s, 2*(y*z-x*w)/s  ],
                     [2*(x*z-y*w)/s,   2*(y*z+x*w)/s,   1-2*(x*x+y*y)/s]
    ])

    return rotm


class Calibrate:

    def __init__(self, tcp_host_ip='172.22.22.2',save_dir=None, workspace_limits=None, calib_point_num=20):
        print "Make sure to roslaunch openni2_launch openni2.launch before running this code!"
        print "Make sure to roslaunch aruco_ros single.launch markerId:=<markerId> markerSize:=<markerSize>"
        
        self.tcp_host_ip = tcp_host_ip
        self.workspace_limits = workspace_limits
        self.save_dir = save_dir

        self.robot = None
        self.aruco = None

        self.markerIncam_pos = None
        self.markerIncam_orn = None
        self.markerIncam_mat = None

        self.calib_point_num = calib_point_num

        # 20 points for calibration
        # Values are obtained from moving the robot in teaching mode
        # and read the value with rob.get_pose() in urx
        self.calib_point = [
            (0.14282, -0.22315, 0.45514, 2.0706478248473146, -1.6414903508903624, 1.0368435382270969),
            (0.15854, -0.19267, 0.44396, 2.1184750445195655, -1.4886177235256481, 1.2713974559693531),
            (0.15331, -0.27321, 0.51774, 2.1471476902138402, -1.4558981541250335, 1.5309999422602492),
            (0.16379, -0.48074, 0.42930, 2.092649080650665, -1.5631709563698295, 1.1499213791083467),
            (0.16385600078638235, -0.48214980394986295, 0.4161398783297443, 2.171314734044116, -1.6708397052569866, 1.0722978920348876),
            (0.04358, -0.46918, 0.45697, 1.8740474732641852, -1.8586896531614672, 1.0172960164477349),
            (-0.11321, -0.38199, 0.40457, 1.4574672485928888, -2.1159781690176627, 1.5411657513520138),
            (-0.12579, -0.37428, 0.41091, 1.4136429710370186, -2.0096733912249447, 1.9069074252588027),
            (0.07445, -0.61316, 0.42427, 2.1086180532042986, -1.9226215431986324, 1.0820013909428459),
            # (0.18917, -0.68770, 0.28746, -2.1136081438009957, 2.0671057104365635, -0.30887843592209752),
            (-0.04003, -0.68773, 0.48108, -1.5515697648058082, 2.0950150892204715, -1.0340024541888257),
            (-0.15762, -0.59901, 0.57168, -1.3416588150098743, 2.2645190536134625, -1.2050594657568403),
            (-0.11677, -0.46207, 0.69365, -1.3161710600247019, 2.3198893891650147, -1.3287794274423772),
            (-0.20012560867488638, -0.5972039219533453, 0.5642451437333025, -1.1152467026827724, 2.0860764881441027, -1.3885137436438686),
            (-0.02950, -0.44617, 0.71450, 1.654160327606111, -2.3211293206781995, 1.165612840260227),
            (0.10531, -0.33048, 0.75721 , 1.9509682164031634, -2.0615572029888862, 1.1078748584393434),
            (0.21782, -0.21388, 0.76401, 2.3485870176455736, -1.615752047097202, 1.0160215961598813),
            (0.03921, -0.29656, 0.79423, 1.2180241325292287, -2.0715233693557717, 0.88950199612736058),
            (-0.20431, -0.55478, 0.59035, 0.68424240279077064, -2.8901015824157592, 0.90132747050124484),
            #(0.43481, -0.51220, 0.40272, 1.4528613310113572, -1.7610822120576275, 0.43000358571424696),
            (0.08307, -0.24159, 0.50617, 1.0228050528025394, -2.0190463059023176, 1.46064680982313),
            (0.08307, -0.24159, 0.50617, 1.0228050528025394, -2.0190463059023176+0.01, 1.46064680982313+0.01)
        ]

        self.robot_poses = []
        self.marker_poses = []

        self.cam2ee = None


    def get_marker_2_cam(self):
        time.sleep(0.5)

        markerIncam_pos, markerIncam_quat, aruco_img = self.aruco.get_pose()

        # If the difference between the previous marker pos and the current is large
        # Consider it got a new detection of tag
        if markerIncam_pos is not None:
            self.markerIncam_pos = markerIncam_pos
            self.markerIncam_orn = quat2rotm(markerIncam_quat)
            self.markerIncam_mat = make_rigid_transformation(self.markerIncam_pos, self.markerIncam_orn)
        else:
            self.markerIncam_mat = None

        return self.markerIncam_mat, aruco_img

    
    def save_transforms_to_file(self, calib_pt_idx, tool_transformation, marker_transformation, aruco_img):
        
        robot_pose_file = os.path.join(self.save_dir, str(calib_pt_idx)  + '_robotpose.txt')
        marker_pose_file = os.path.join(self.save_dir, str(calib_pt_idx) + '_markerpose.txt')
        aruco_img_file = os.path.join(self.save_dir, str(calib_pt_idx) + '_img.png')
        
        # Tool pose in robot base frame
        with open(robot_pose_file, 'w') as file1:
            for l in np.reshape(tool_transformation, (16, )).tolist():
                file1.writelines(str(l) + ' ')

        # Marker pose in camera frame
        with open(marker_pose_file, 'w') as file2:
            for l in np.reshape(marker_transformation, (16, )).tolist():
                file2.writelines(str(l) + ' ')
        
        cv2.imwrite(aruco_img_file, aruco_img)


    def collect_data(self):
        """Collect data for calibration
        """
        # Initialize the robot
        self.robot = Robot(self.workspace_limits, self.tcp_host_ip)
        
        # Initialize the aruco
        self.aruco = ArUco()
        rospy.init_node('aruco', anonymous=True)
        time.sleep(0.5)
        
        epsilon = 0.05 # randomness for sampling orientation
        complete_point_num = 0
        
        while complete_point_num < self.calib_point_num:

            pos = self.calib_point[complete_point_num][:3]
            orn = self.calib_point[complete_point_num][3:]
            
            # Move the robot
            robot_transform = self.robot.move_to(pos, orn)
            time.sleep(2)

            # Marker Pose
            marker_pose, aruco_img = self.get_marker_2_cam()

            if marker_pose is not None:
                # Robot Pose
                rotm = robot_transform.orient.array
                pos = robot_transform.pos.array
                robot_pose = make_rigid_transformation(pos, rotm)

                print("Get the marker in frame!")
                print(self.calib_point[complete_point_num])
                # print("Robot Pose")
                # print(robot_pose)
                # print("Marker Pose")
                # print(marker_pose)
                print("#################")
                self.save_transforms_to_file(complete_point_num, robot_pose, marker_pose, aruco_img)

                complete_point_num += 1
            
            time.sleep(1)

    
    def load_transforms(self, load_dir):
        """ Load robot pose and marker pose from a save directory
        # Arguments
        load_dir: the directory where calibration data was previously acquired from the robot.
        # Returns
        Two lists of 4x4 transformation matrices.
        self.robot_poses, self.marker_poses
        """

        for f in os.listdir(load_dir):
            if 'robotpose.txt' in f:
                robot_pose_file = f
                marker_pose_file = f[:-13] + 'markerpose.txt'

                # tool pose in robot base frame
                with open(os.path.join(load_dir, robot_pose_file), 'r') as file_robot:
                    robotpose_str = file_robot.readline().split(' ')
                    robotpose = [float (x) for x in robotpose_str if x is not '']
                    assert len(robotpose) == 16
                    robotpose = np.reshape(np.array(robotpose), (4, 4))
                self.robot_poses.append(robotpose)

                # marker pose in camera frame
                with open(os.path.join(load_dir, marker_pose_file), 'r') as file_marker:
                    markerpose_str = file_marker.readline().split(' ')
                    markerpose = [float(x) for x in markerpose_str if x is not '']
                    assert len(markerpose) == 16
                    markerpose = np.reshape(np.array(markerpose), (4, 4))
                self.marker_poses.append(markerpose)
    

    def axxb(self):
        """
        AX=XB solver.
        
        Args:
        - self.robot_poses (list of 4x4 numpy array): poses (homogenous transformation) of the robot end-effector in the robot base frame.
        - self.marker_poses (list of 4x4 numpy array): poses (homogenous transformation) of the marker in the camera frame.

        Return:
        - self.cam2ee (4x4 numpy array): poses of the camera in the robot end-effector frame.
        """

        assert len(self.robot_poses) == len(self.marker_poses), 'robot poses and marker poses are not of the same length!'

        n = len(self.robot_poses)
        pose_inds= np.arange(n)
        np.random.shuffle(pose_inds)

        print "Total Pose: %i" % n
        A = np.zeros((4, 4, n-1))
        B = np.zeros((4, 4, n-1))
        alpha = np.zeros((3, n-1))
        beta = np.zeros((3, n-1))

        M = np.zeros((3, 3))

        for i in range(n-1):
            A[:, :, i] = np.matmul(pose_inv(self.robot_poses[pose_inds[i+1]]), self.robot_poses[pose_inds[i]])
            B[:, :, i] = np.matmul(self.marker_poses[pose_inds[i+1]], pose_inv(self.marker_poses[pose_inds[i]]))
            alpha[:, i] = get_mat_log(A[:3, :3, i])
            beta[:, i] = get_mat_log(B[:3, :3, i])
            M += np.outer(beta[:, i], alpha[:, i])

            # Bad pair of transformation are very close in the orientation.
            # They will give nan result
            if np.sum(np.isnan(alpha[:, i])) + np.sum(np.isnan(beta[:, i])) > 0:
                nan_num += 1
                continue
            else:
                M += np.outer(beta[:, i], alpha[:, i])

        # Get the rotation matrix
        mtm = np.matmul(M.T, M)
        u_mtm, s_mtm, vh_mtm = np.linalg.svd(mtm)
        R = np.matmul(np.matmul(np.matmul(u_mtm, np.diag(np.power(s_mtm, -0.5))), vh_mtm), M.T)

        # Get the tranlation vector
        I_Ra_Left = np.zeros((3*(n-1), 3))
        ta_Rtb_Right = np.zeros((3 * (n-1), 1))
        for i in range(n-1):
            I_Ra_Left[(3*i):(3*(i+1)), :] = np.eye(3) - A[:3, :3, i]
            ta_Rtb_Right[(3*i):(3*(i+1)), :] = np.reshape(A[:3, 3, i] - np.dot(R, B[:3, 3, i]), (3, 1))
        

        t = np.linalg.lstsq(I_Ra_Left, ta_Rtb_Right, rcond=-1)[0]
        
        self.cam2ee = np.c_[R, t]
        self.cam2ee = np.r_[self.cam2ee, [[0, 0, 0, 1]]]

        print "Calibration Result:\n", self.cam2ee
    

    def test(self):
        """ Test the accuracy of the calculated result.
            Use AXB to calculate the base2tag transformation for each frame.
        """
        n = len(self.robot_poses)
        for i in range(n):
            base2tag = np.matmul(np.matmul(self.robot_poses[i], self.cam2ee), self.marker_poses[i])
            print("base2tag #{}".format(i))
            print(base2tag)

    
    def calibrate(self):

        self.load_transforms(self.save_dir)
        self.axxb()
        self.test()

        # Save the calibration file
        calibration_file = os.path.join(self.save_dir, 'camera_pose.txt')
        with open(calibration_file, 'w') as file1:
            for l in np.reshape(self.cam2ee, (16, )).tolist():
                file1.writelines(str(l) + ' ')

        self.robot.disconnect()


if __name__ == "__main__":
    workspace_limits = [[0.3, -0.3], [-0.4, -0.6], [0.3, 0.5]]
    save_dir = "/home/hongtao/Desktop/0818_calib"
    if not os.path.exists(save_dir):
        os.mkdir(save_dir)
    C = Calibrate(workspace_limits=workspace_limits, save_dir=save_dir)
    C.collect_data()
    C.calibrate()

