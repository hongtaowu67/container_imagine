#! /usr/bin/env python

from aruco import ArUco
import numpy as np
import rospy
import time

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


def make_rigid_transformation(pos, orn):
    """
    Rigid transformation from position and orientation.

    Args:
    - pos (3, numpy array): translation
    - orn (4, numpy array): orientation in quaternion

    Returns:
    - homo_mat (4x4 numpy array): homogenenous transformation matrix
    """
    rotm = quat2rotm(orn)
    homo_mat = np.c_[rotm, np.reshape(pos, (3, 1))]
    homo_mat = np.r_[homo_mat, [[0, 0, 0, 1]]]
    
    return homo_mat



class Calibrate:
    def __init__(self):
        print "Make sure to roslaunch openni2_launch openni2.launch before running this code!"
        print "Make sure to roslaunch aruco_ros single.launch markerId:=<markerId> markerSize:=<markerSize>"
        
        self.aruco = ArUco()
        rospy.init_node('aruco', anonymous=True)
        time.sleep(0.5)

        self.markerIncam_pos = None
        self.markerIncam_orn = None
        self.markerIncam_mat = None
        
        self.ee2base_pos = None
        self.ee2base_orn = None
        self.ee2base_mat = None
    

    def get_marker_2_cam(self):
        time.sleep(0.5)

        self.markerIncam_pos, self.markerIncam_orn = self.aruco.get_pose()

        if self.markerIncam_pos is not None:
            self.markerIncam_mat = make_rigid_transformation(self.markerIncam_pos, self.markerIncam_orn)
        else:
            self.markerIncam_mat = None
        
        if self.markerIncam_mat is not None:
            print('Marker in Cam')
            print(self.markerIncam_mat)
            print('Cam in Marker')
            print(np.linalg.inv(self.markerIncam_mat))
            print("#####")




if __name__ == "__main__":
    C = Calibrate()

    while True:
        C.get_marker_2_cam()