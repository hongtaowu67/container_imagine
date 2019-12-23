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


class Robot(object):

    def __init__(self, workspace_limits=None, tcp_host_ip=None, calibrate=False):
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
        self.acc = 1
        self.vel = 1

        # Default home joint configuration
        self.home_config = (0.03263, -0.31182, 0.49574, 
                0.85278977222806773, -2.2014136950986072, 1.2277969610053945)
        
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
        self.robot.movel(self.home_config, self.acc, self.vel)


# Test
if __name__ == "__main__":
    workspace_limits = [[0.2, 0.2], [-0.5, -0.8], [0.4, 0.5]]
    robot = Robot(workspace_limits=workspace_limits, tcp_host_ip='172.22.22.2')