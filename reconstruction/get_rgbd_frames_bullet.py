#! /usr/bin/env python
"""
[WIP]
This code is written for getting the rbgd frames for tsdf-fusion.
@author: Hongtao Wu
Nov 03, 2019
"""

import pybullet as p
import pybullet_data
import numpy as np
import cv2
import math


class RGBDFrame(object):
  
  def __init__(self, obj_urdf, obj_orn=[0, 0, 0],frame_num=50):
    """
    Args:
    - obj_orn: the orientation of the object in Euler angle
    - frame_num: number of frame to take, it should be a multiple of 10
    """
    super(RGBDFrame, self).__init__()

    physical_client = p.connect(p.GUI)
    p.setGravity(0, 0, -10)

    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    # Load plane
    self.plane_id = p.load("plane.urdf")

    # Load table: the table is already in the right pos and orn
    self.table_urdf = "../object/table/table.urdf"
    self.table_id = p.loadURDF(self.table_urdf)

    # Load object
    self.obj_id = p.loadURDF(obj_urdf)

    table_aabb = p.getAABB(self.table_id)
    obj_aabb = p.getAABB(self.obj_id)

    # Raise the object up
    p.resetBasePositionAndOrientation(self.obj_id, \
        posObj=(0, 0, table_aabb[1][2]-obj_aabb[1][2]), \
        ornObj=p.getQuaternionFromEuler(obj_orn))

    # Let the object rest on the table
    for i in range(100):
      p.stepSimulation()

    obj_pos, _ = p.getBasePositionAndOrientation(self.obj_id)

    # Set up camera
    self.cam_target_pos = obj_pos
    self.pixel_width = 640
    self.pixel_height = 480
    self.aspect_ratio = self.pixel_width / self.pixel_height
    self.near_plane = 0.05
    self.far_plane = 10
    self.fov = 60
    # The camera will be rotating around the object 
    self.cam_distance = 0.6  # the camera is 60cm away from the object
    self.cam_roll = 0.0
    self.camera_yaw = np.linspace(0, math.pi*2, 10)
    self.camera_pitch = np.linspace(0, math.pi/2, int(frame_num/10))


if __name__ == "__main__":
  RBGDFrame = RGBDFrame()

