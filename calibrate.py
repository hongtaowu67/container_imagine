#! /usr/bin/env python

from calibrate.aruco import ArUco
import numpy as np
import rospy
import time
from robot import Robot
from utils import make_rigid_transformation, quat2rotm

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

    def __init__(self, tcp_host_ip='172.22.22.2',save_dir=None, workspace_limits=None, calib_point_num=30):
        print "Make sure to roslaunch openni2_launch openni2.launch before running this code!"
        print "Make sure to roslaunch aruco_ros single.launch markerId:=<markerId> markerSize:=<markerSize>"
        
        self.workspace_limits = workspace_limits

        self.robot = Robot(self.workspace_limits, tcp_host_ip)
        
        self.aruco = ArUco()
        rospy.init_node('aruco', anonymous=True)
        time.sleep(0.5)

        self.markerIncam_pos = None
        self.markerIncam_orn = None

        # Place the tag at a position where the camera can see the tag at home position
        self.markerIncam_pos, _ = self.aruco.get_pose()

        self.calib_point_num = calib_point_num

        # Values are obtained from moving the robot in teaching mode
        # and read the value with rob.get_pose() in urx
        self.calibration_orn = [[[2.0849771838777822, 1.154796711201963, -0.27191177786048848], 
                                [1.7050381847811928, 1.6784915484788332, -0.57787071188056516], 
                                [1.6505922172425964, 1.7685039622413643, -0.31499784157645533],
                                [1.8266154616606618, 2.0394733857552945, -0.34732284534340724]], 
                               [[2.1363630736354229, 1.2316614839213429, 0.18869285661438978], 
                                [2.1686120566833802, 1.5133539396258169, 0.21500147577327908], 
                                [1.9346763255710158, 1.246221227551646, 0.34526209695370191]],
                                [2.0556688938020238, 1.4044727967099233, 0.35105867311343597]]


    def get_marker_2_cam(self):
        time.sleep(0.5)

        markerIncam_pos, markerIncam_quat = self.aruco.get_pose()

        # If the difference between the previous marker pos and the current is large
        # Consider it got a new detection of tag
        if (np.linalg.norm(markerIncam_pos - self.markerIncam_pos) > 0.0001):
            self.markerIncam_pos = markerIncam_pos
            self.markerIncam_orn = quat2rotm(markerIncam_quat)
            self.markerIncam_mat = make_rigid_transformation(self.markerIncam_pos, self.markerIncam_orn)
        else:
            self.markerIncam_mat = None
        
        # if self.markerIncam_mat is not None:
        #     print('Marker in Cam')
        #     print(self.markerIncam_mat)
        #     print('Cam in Marker')
        #     print(np.linalg.inv(self.markerIncam_mat))
        #     print("#####")

        return self.markerIncam_mat


    def collect_data(self):
        """Collect data for calibration
        """
        
        epsilon = 0.05 # randomness for sampling orientation
        complete_point_num = 0
        
        while complete_point_num < self.calib_point_num:
            # Sample a position winthin the workspace
            pos = [self.workspace_limits[0][0] + np.random.rand() * (self.workspace_limits[0][1] - self.workspace_limits[0][0]),
                   self.workspace_limits[1][0] + np.random.rand() * (self.workspace_limits[1][1] - self.workspace_limits[1][0]),
                   self.workspace_limits[2][0] + np.random.rand() * (self.workspace_limits[2][1] - self.workspace_limits[2][0])]
            
            # Sample a random orientation
            if pos[0] >= 0:
                np.random.shuffle(self.calibration_orn[0])
                orn = self.calibration_orn[0][0]
            else:
                np.random.shuffle(self.calibration_orn[1])
                orn = self.calibration_orn[1][0]

            orn[0] += np.random.rand() * epsilon * (0.5 - np.random.rand())
            orn[1] += np.random.rand() * epsilon * (0.5 - np.random.rand())
            orn[2] += np.random.rand() * epsilon * (0.5 - np.random.rand())
            
            # Move the robot
            robot_transform = self.robot.move_to(pos, orn)
            time.sleep(1)

            # Marker Pose
            marker_pose = self.get_marker_2_cam()

            if marker_pose is not None:
                # Robot Pose
                rotm = robot_transform.orient.array
                pos = robot_transform.pos.array
                robot_pose = make_rigid_transformation(pos, rotm)

                print("Get the marker in frame!")
                print("Robot Pose")
                print(robot_pose)
                print("Marker Pose")
                print(marker_pose)
                print("#################")
                time.sleep(1)

                complete_point_num += 1


if __name__ == "__main__":
    workspace_limits = [[0.3, -0.3], [-0.4, -0.6], [0.3, 0.5]]
    C = Calibrate(workspace_limits=workspace_limits)
    C.collect_data()
