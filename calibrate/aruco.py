"""
Get the transformation from marker to the camera.
@author: Hongtao Wu
Nov 15, 2019
"""

import rospy
import roslib
from geometry_msgs.msg import PoseStamped

import numpy as np

class ArUco:
    def __init__(self):
        self.pose_topic = "/aruco_single/pose"
        
        self.pose_x = None
        self.pose_y = None
        self.pose_z = None
        self.pose_qw = None
        self.pose_qx = None
        self.pose_qy = None
        self.pose_qz = None
        
        self.pos = None
        self.orn = None

        # Flags for message publishing
        self.get_new_msg = False
        self.msg_registered = False
        
        self._pose_sub = rospy.Subscriber(self.pose_topic, PoseStamped, self._poseInfoCb)

    def _poseInfoCb(self, msg):
        if msg is None:
            rospy.logwarn("_poseInfoCb: msg is None!")

        self.get_new_msg = True
        self.msg_registered = False
        
        self.pose_x = msg.pose.position.x
        self.pose_y = msg.pose.position.y
        self.pose_z = msg.pose.position.z
        self.pose_qx = msg.pose.orientation.x
        self.pose_qy = msg.pose.orientation.y
        self.pose_qz = msg.pose.orientation.z
        self.pose_qw = msg.pose.orientation.w
        self.pos = np.array([self.pose_x, self.pose_y, self.pose_z])
        self.orn = np.array([self.pose_qw, self.pose_qx, self.pose_qy, self.pose_qz])

    def get_pose(self):
        pos = None
        orn = None

        # If new message received, then publish. If no message received, publish none.
        if (self.get_new_msg) and (not self.msg_registered):
            if self.pos is not None:
                pos = self.pos.copy()
            if self.orn is not None:
                orn = self.orn.copy()
            self.get_new_msg = False
            self.msg_registered = True

        return pos, orn



