"""
This script imagines pouring from a container onto the object.
Find the best pouring spot and angle for pouring into a specific object.

Author: Hongtao Wu
March 28, 2020
"""

from __future__ import division

import numpy as np
import pybullet as p
import pybullet_data
import os
import time
import math
import trimesh

bottle_urdf = "/home/hongtao/Dropbox/ICRA2021/data/general/bottle/JuiceBottle_GeoCenter.urdf"
content_urdf = "/home/hongtao/Dropbox/ICRA2021/data/general/m&m.urdf"

class BottlePour(object):
    def __init__(self, bottle_urdf, content_urdf, obj_urdf, pour_pos, check_process=True,
                 ):
        """
        Args:
        -- bottle_obj: the urdf of the pouring bottle
        -- obj_urdf: the urdf of the object being poured
        -- pour_pos (np array in [x y z]): the position of the pour point
        """
        super(BottlePour, self).__init__()
        
        if check_process:
            self.pysical_client = p.connect(p.GUI)
        else:
            self.physical_client = p.connect(p.DIRECT)
        self.simulation_iteration = 1000
        self.check_process = check_process
        p.setGravity(0, 0, -9.81)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.plane_id = p.loadURDF("plane.urdf")

        # Bottle
        self.bottle_id = p.loadURDF(bottle_urdf)
        self.pour_angle = [0, -np.pi/20, -np.pi/4, -2*np.pi/5]
        self.pour_pos = pour_pos
        self.bottle_pos = np.array([0.0, 0.0, 0.0])
        p.changeDynamics(self.bottle_id, -1, mass=1)
        self.bottle_aabb = p.getAABB(self.bottle_id)

        # Content
        self.content_urdf = content_urdf
        self.content_num = 80
        self.content_id_list = []
        content_aabb_id = p.loadURDF(self.content_urdf)
        self.content_aabb = p.getAABB(content_aabb_id)
        p.removeBody(content_aabb_id)
        self.content_restitution = 0.0
        self.content_lateralfriction = 0.0

        # Object
        self.obj_id = p.loadURDF(obj_urdf)
        self.obj_raise = 0.1
        self.obj_zero_orn = p.getQuaternionFromEuler([0, 0, 0])
        self.obj_zero_pos = [0, 0, 0]
        self.obj_curr_aabb = p.getAABB(self.obj_id)
        
        # # Raise up the object (object is 0.1m above the plane)
        # if self.obj_curr_aabb[0][2] <= 0.1:
        #     p.resetBasePositionAndOrientation(self.obj_id, 
        #             posObj=(0, 0, -self.obj_curr_aabb[0][2]+0.1),
        #             ornObj=self.obj_zero_orn)

        #     # Raise the pouring pos as well
        #     self.pour_pos[-1] += -self.obj_curr_aabb[0][2]+0.1

        #     self.obj_curr_aabb = p.getAABB(self.obj_id)
        #     self.obj_zero_pos = [0, 0, -self.obj_curr_aabb[0][2]+0.1]
        
        # Create constraint on the cup to fix its position
        p.changeDynamics(self.obj_id, -1, mass=1)
        self.constraint_Id = p.createConstraint(self.obj_id, -1, -1, -1, p.JOINT_FIXED, jointAxis=[0, 0, 0],
                parentFramePosition=[0, 0, 0], childFramePosition=[0, 0, 0]) 


    def rotate_bottle(self, planar_angle, indent=0.01):
        """
        Rotate the bottle about the pour_pos.
        """

        bottle_curr_aabb = p.getAABB(self.bottle_id)
        bottle_half_length = bottle_curr_aabb[1][0] + indent # Adding indent to give a little offset for pouring
        pour_pos_offset = - bottle_half_length * np.array([np.cos(planar_angle), np.sin(planar_angle)])
        self.bottle_pos[0] = self.pour_pos[0] + pour_pos_offset[0]
        self.bottle_pos[1] = self.pour_pos[1] + pour_pos_offset[1]
        self.bottle_pos[2] = self.pour_pos[2] + (-self.bottle_aabb[0][2] - 0.015) # offset for the tip of the bottle

        p.resetBasePositionAndOrientation(self.bottle_id,
                                          posObj=self.bottle_pos,
                                          ornObj=p.getQuaternionFromEuler([0, 0, planar_angle]))

        # parentFramePosition: the joint frame pos in the object frame
        # childFramePosition: the joint frame pos in the world frame if the child frame is set to be -1 (base)
        bottle_constraint_Id = p.createConstraint(self.bottle_id, -1, -1, -1, p.JOINT_FIXED, jointAxis=[0, 0, 0],
                parentFramePosition=(self.pour_pos-self.bottle_pos), childFramePosition=self.pour_pos)

        self.load_content(planar_angle)
        
        pivot = self.pour_pos
        for i in range(self.simulation_iteration):
            p.stepSimulation()

            if self.check_process:
                time.sleep(1. / 240.)           

            orn = p.getQuaternionFromEuler([0, 2*np.pi/5 * math.sin(math.pi * 2 * (i) / int(4 * self.simulation_iteration)), 0])

            # TODO: Add rotation to the object instead of the bottle to pour from different angle
            p.changeConstraint(bottle_constraint_Id, pivot, jointChildFrameOrientation=orn, maxForce=50)


    def load_content(self, planar_angle):
        """
        Load contents into the bottle.
        """

        # Contents are loaded at the middle between the bottle coneter and the bottle bottom
        content_pos = self.bottle_pos
        # content_pos[0] += 0.5 * self.bottle_aabb[0][0] * np.cos(planar_angle)
        # content_pos[1] += 0.5 * self.bottle_aabb[0][0] * np.sin(planar_angle)
        x_range = np.abs(self.bottle_aabb[0][0]) - np.abs(self.content_aabb[1][0] - self.content_aabb[0][0]) * 2
        y_range = np.abs(self.bottle_aabb[1][1] - self.bottle_aabb[0][1]) - np.abs(self.content_aabb[1][1] - self.content_aabb[0][1]) * 4
        z_range = np.abs(self.bottle_aabb[1][2] - self.bottle_aabb[0][2]) - np.abs(self.content_aabb[1][2] - self.content_aabb[0][2]) * 4

        x_num_range = np.floor(x_range / np.abs(self.content_aabb[1][0] - self.content_aabb[0][0]))
        y_num_range = np.floor(y_range / np.abs(self.content_aabb[1][1] - self.content_aabb[0][1]))
        z_num_range = np.floor(z_range / np.abs(self.content_aabb[1][2] - self.content_aabb[0][2]))

        for i in range(self.content_num):
            content_id = p.loadURDF(content_urdf)
            self.content_id_list.append(content_id)

            x_offset = np.random.random_integers(x_num_range) / x_num_range * x_range
            y_offset = np.random.random_integers(y_num_range) / y_num_range * y_range + self.bottle_aabb[0][1] - self.content_aabb[0][1] * 2
            z_offset = np.random.random_integers(z_num_range) / z_num_range * z_range + self.bottle_aabb[0][2] - self.content_aabb[0][2] * 2

            x_offset_angle =  np.cos(planar_angle) * x_offset + np.sin(planar_angle) * y_offset
            y_offset_angle = -np.sin(planar_angle) * x_offset + np.cos(planar_angle) * y_offset

            p.resetBasePositionAndOrientation(content_id,
                                              posObj=(content_pos[0] - x_offset_angle,
                                                      content_pos[1] + y_offset_angle,
                                                      content_pos[2] + z_offset),
                                              ornObj=self.obj_zero_orn)
            
            p.changeDynamics(content_id, -1, restitution=self.content_restitution, 
                lateralFriction=self.content_lateralfriction,
                spinningFriction=0.5,
                rollingFriction=0.5)

        # Let the sphere to drop
        for i in range(100):
            p.stepSimulation()



    def disconnect_p(self):
        p.disconnect()



if __name__ == "__main__":
    obj_urdf = "/home/hongtao/Dropbox/ICRA2021/data/training_set/Container/Blue_Cup/Blue_Cup_mesh_0.urdf"
    pour_pos = np.array([-0.08763265508145837, -0.26190104459071484, 0.19611099708080304])
    BP = BottlePour(bottle_urdf, content_urdf, obj_urdf, pour_pos)
    BP.rotate_bottle(np.pi/2)
    BP.disconnect_p()


    