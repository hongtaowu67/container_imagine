"""
Robot control and action
Forked from Andy Zeng's visual-pushing-grasping: https://github.com/andyzeng/visual-pushing-grasping

Hongtao Wu
Dec 22, 2019
"""

import socket
import select
import struct
import time
import os
import numpy as np
import itertools
import urx

from utils import make_rigid_transformation


class Robot(object):

    def __init__(self, workspace_limits=None, tcp_host_ip='172.22.22.2', calibrate=False, acc=1, vel=1):
        '''
        UR5 Robot

        workspace_limits (3x2 float array): [[x_min, x_max], [y_min, y_max], [z_min, z_max]]
        tcp_host_ip (string): tcp to connect to the robot
        calibrate (bool): to calibration or not
        '''

        print("Initializing the robot...")
        # Connect to robot client
        self.tcp_host_ip = tcp_host_ip
        self.robot = urx.Robot(tcp_host_ip)

        # Acceleration and veloctiy
        self.acc = acc
        self.vel = vel

        # Default home joint configuration
        self.home_config = (0.8367955088615417, -1.8993938604937952, 1.2887015342712402, -1.6408045927630823, -1.0915311018573206, -1.3117168585406702)
        
        # Sleep time between different frame
        self.sleep_time = 0.5

        self.go_home()

        #TODO: Initialize the camera

    
    def move_to(self, tool_position, tool_orientation=None):
        """Move robot to position and orientation

        Args:
        - tool_position (list): x, y, z
        - tool_orientation (list): rx, ry, rz (in the axis-angle UR5 tradition)
        """
        if tool_orientation is None:
            config = tuple(tool_position + [0, np.pi, 0])
            transform = self.robot.movel(config, self.acc, self.vel)
        else:
            config = tuple(tool_position + tool_orientation)
            transform = self.robot.movel(config, self.acc, self.vel)
        
        time.sleep(self.sleep_time)

        return transform
    

    def go_home(self):
        self.robot.movej(self.home_config, self.acc, self.vel)


    def get_pose(self):
        """Return a 4x4 numpy array rigid body transformation
        """

        xform = self.robot.get_pose()
        pos = xform.pos.array
        rotm = xform.orient.array

        return make_rigid_transformation(pos, rotm)


    def move_to_joint(self, joint_config):
        """Move in joint configuration

        Args:
        - joint_config (6, tuple): 6 joints for UR5 robot
        """
        self.robot.movej(joint_config, self.acc, self.vel)

    
    def disconnect(self):
        """Disconnect connection to the robot
        """
        self.robot.close()



# # Test
# if __name__ == "__main__":
#     workspace_limits = [[0.2, 0.2], [-0.5, -0.8], [0.4, 0.5]]
#     robot = Robot(workspace_limits=workspace_limits, tcp_host_ip='172.22.22.2')