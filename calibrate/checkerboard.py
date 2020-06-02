"""
Get the transformation of the checkerboard in the camera frame.
Author: Hongtao Wu
June 03, 2020
"""

import rospy
import roslib
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image

import message_filters
import numpy as np
import time
import cv2

from cv_bridge import CvBridge
from cv_bridge import CvBridgeError


class Checkerboard:
    
    def __init__(self):
        self.pose_topic = "/capture/pose_check"
        self.result_topic = "/capture/pose_result"
    
        self.pose_sub = message_filters.Subscriber(self.pose_topic, PoseStamped)
        self.result_sub = message_filters.Subscriber(self.result_topic, Image)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.pose_sub, self.result_sub], 10, 0.1, allow_headerless=False)
        self.ts.registerCallback(self._callback)

        self.pose_x = None
        self.pose_y = None
        self.pose_z = None
        self.pose_qw = None
        self.pose_qx = None
        self.pose_qy = None
        self.pose_qz = None
        
        self.pos = None
        self.orn = None

        self.result = None

        self._bridge = CvBridge()


    def _callback(self, msg_pose, msg_result):
        if msg_pose is None:
            rospy.logwarn("pose msg is None!")
        if msg_result is None:
            rospy.logwarn("result msg is None!")

        # Update pose (orn in quaternion)
        self.pose_x = msg_pose.pose.position.x
        self.pose_y = msg_pose.pose.position.y
        self.pose_z = msg_pose.pose.position.z
        self.pose_qx = msg_pose.pose.orientation.x
        self.pose_qy = msg_pose.pose.orientation.y
        self.pose_qz = msg_pose.pose.orientation.z
        self.pose_qw = msg_pose.pose.orientation.w
        self.pos = np.array([self.pose_x, self.pose_y, self.pose_z])
        self.orn = np.array([self.pose_qw, self.pose_qx, self.pose_qy, self.pose_qz])

        # Updaste images    
        cv_image = self._bridge.imgmsg_to_cv2(msg_result, "passthrough")
        self.result = cv_image


    def get_pose(self):
        return self.pos, self.orn, self.result



