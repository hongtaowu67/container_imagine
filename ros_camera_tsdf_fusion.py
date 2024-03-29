# Capture different color and depth frames and the corresponding pose of the camera with ROS.

# Author: Hongtao Wu
# Institution: Johns Hopkins University
# Date: Nov 24, 2019

from __future__ import print_function

import os
import time

import rospy
import roslib
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
from geometry_msgs.msg import PoseStamped

import numpy as np
import cv2
from threading import Lock

from utils import make_rigid_transformation


class ROSCameraTSDFFusion:
    """
    Class to capture data for TSDF fusion with ROS
    """
    def __init__(self, automatic=False):
        """
        @type  automatic: bool
        @param automatic: if True, then use robot transformation to get camera pose;
            if False, then use ArUco tag to get camera pose
        """

        rospy.loginfo("Initializing ROSCameraTSDFFusion...")
        rospy.loginfo(
            "Make sure to roslaunch openni2_launch openni2.launch depth_registration:=true"
        )

        self.automatic = automatic

        self.rgb_topic = '/camera/rgb/image_rect_color'
        self.depth_topic = '/camera/depth_registered/hw_registered/image_rect'

        self.rgb_img = None
        self.depth_img = None
        self._rgb_sub = rospy.Subscriber(self.rgb_topic, Image, self._rgbCb)
        self._depth_sub = rospy.Subscriber(self.depth_topic, Image,
                                           self._depthCb)

        if not self.automatic:
            self.aruco_pose_topic = '/aruco_single/pose'
            self.camera_pose = None
            self._pose_sub = rospy.Subscriber(self.aruco_pose_topic,
                                              PoseStamped, self._poseCb)

        self._bridge = CvBridge()

        self.mutex = Lock()

        rospy.loginfo("Finish initializing ROSCameraTSDFFusion...")

    def _rgbCb(self, msg):
        if msg is None:
            rospy.logwarn('_rgbCb: msg is None!')
        try:
            cv_img = self._bridge.imgmsg_to_cv2(msg, 'rgb8')
            cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)

            self.rgb_img = cv_img

        except CvBridgeError as e:
            rospy.logwarn(str(e))

    def _depthCb(self, msg):
        if msg is None:
            rospy.logwarn('_depthCb: msg is None!')
        try:
            cv_img = self._bridge.imgmsg_to_cv2(msg, 'passthrough')

            self.depth_img = cv_img

        except CvBridgeError as e:
            rospy.logwarn(str(e))

    def _poseCb(self, msg):
        if msg is None:
            rospy.logwarn('_depthCb: msg is None!')
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        qw = msg.pose.orientation.w
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z

        pos = np.array([x, y, z])
        orn = np.array([qw, qx, qy, qz])

        # Camera pose in marker frame
        self.camera_pose = np.linalg.inv(make_rigid_transformation(pos, orn))

    def get_frame(self):
        rgb_img = None
        depth_img = None
        camera_pose = None

        with self.mutex:
            if self.rgb_img is not None:
                rgb_img = self.rgb_img
            else:
                print('rgb_img is None! Returning None!')
                return None, None, None

            if self.depth_img is not None:
                depth_img = self.depth_img
            else:
                print('depth_img is None! Returning None!')
                return None, None, None

            # If automatic, then camera_pose = None
            if not self.automatic:
                if self.camera_pose is not None:
                    camera_pose = self.camera_pose
                else:
                    print('camera_pose is None! Returning None!')
                    return None, None, None

            return rgb_img, depth_img, camera_pose