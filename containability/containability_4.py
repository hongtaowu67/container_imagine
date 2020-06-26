#! /usr/bin/env python

"""
This script is written to check whether an object is able to contain sphere.
No physics about the cup is reasoned. This scrip only test the containability
This is written for cup imagination project.
@author: Hongtao Wu 
Nov 02, 2019

Modify for using shaking the objects instead of applying horizontal force field.
Modifier: Hongtao Wu
Dec 29, 2019

Modify for running in python 2.7
Modifier: Hongtao
Feb 11, 2020

Modify for using OBB as the dropping area
"""
from __future__ import division

import numpy as np
import pybullet as p
import pybullet_data
import time
import os
import math
import trimesh


sphere_urdf = "/home/hongtao/Dropbox/ICRA2021/data/general/m&m.urdf"

class Containability(object):
    def __init__(self, obj_urdf, obj_vhacd_mesh, obj_zero_pos=[0, 0, 1], obj_zero_orn=[0, 0, 0], 
                 check_process=False, mp4_dir=None, object_name=None,
                 content_urdf=sphere_urdf):
        """
        Args:
        - obj_zero_orn: the start orientation of the object in Euler Angle
        - check_process: if set True, the process will be real time
        """
        super(Containability, self).__init__()
        # Hyperparameter
        self.sphere_num_max = 250
        self.sphere_num_min = 80
        self.sphere_in_percentage_threshold = 0.08
        self.sphere_urdf = content_urdf
        self.sphere_in_percentage = 0.0
        self.sphere_x_range = np.nan
        self.sphere_y_range = np.nan
        self.drop_sphere_num = 0
        self.x_sphere_num = 0
        self.y_sphere_num = 0
        self.sphere_dist_scale = 1

        # Restitution
        self.sphere_restitution = 0.1
        self.object_restitution = 0.1
        self.plane_restitution = 0.1

        self.obj_urdf = obj_urdf
        self.containability = False

        # Simulation Parameter
        self.simulation_iteration = 1500
        self.check_process = check_process

        # Sphere Information
        self.sphere_id = []
        self.sphere_drop_orn = p.getQuaternionFromEuler([0, 0, 0])
        self.sphere_drop_pos = []
        self.sphere_in_drop_pos = []
        self.sphere_drop_z = 0
        self.sphere_lateralfriction=0.005

        # Set the world
        if not check_process:
            self.physicsClient = p.connect(p.DIRECT)
        else:
            self.physicsClient = p.connect(p.GUI)
        p.setGravity(0, 0, -9.81)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        # Save mp4 video
        if mp4_dir is not None:
            self.save_mp4_dir = mp4_dir
            self.object_name = object_name
            mp4_file_name = self.object_name + ".mp4"
            mp4_file_path = os.path.join(self.save_mp4_dir, mp4_file_name)
            p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, mp4_file_path)

        p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)

        # Reset debug camera postion
        p.resetDebugVisualizerCamera(1.0, 0, -44, [-0.05, -0.1, 1])

        # Load plane
        self.plane_id = p.loadURDF("plane.urdf")
        p.changeDynamics(self.plane_id, -1, restitution=self.plane_restitution)

        # Load object
        self.obj_zero_pos = obj_zero_pos
        self.obj_zero_orn = p.getQuaternionFromEuler(obj_zero_orn) # Quaternion
        self.obj_id = p.loadURDF(self.obj_urdf, basePosition=self.obj_zero_pos, baseOrientation=self.obj_zero_orn, 
                useFixedBase=True)
        p.changeDynamics(self.obj_id, -1, restitution=self.object_restitution)

        # Get the bounding box of the cup
        self.obj_curr_aabb = p.getAABB(self.obj_id)

        # Create constraint on the cup to fix its position
        p.changeDynamics(self.obj_id, -1, mass=1)
        self.constraint_Id = p.createConstraint(self.obj_id, -1, -1, -1, p.JOINT_FIXED, jointAxis=[0, 0, 0],
                parentFramePosition=[0, 0, 0], childFramePosition=self.obj_zero_pos) 

        # Get the Four points of the OBB
        self.obj_vhacd_mesh = trimesh.load(obj_vhacd_mesh)
        self.obj_obb_transform = self.obj_vhacd_mesh.bounding_box_oriented.primitive.transform
        self.obj_obb_extents = self.obj_vhacd_mesh.bounding_box_oriented.primitive.extents
        self.obj_obb_center = self.obj_obb_transform[:3, -1]
        
        # Half length of the obb
        self.x_range = self.obj_obb_extents[0]/2
        self.y_range = self.obj_obb_extents[1]/2
        self.z_range = self.obj_obb_extents[2]/2

        # obb axes in the world frame
        self.x_axis = self.obj_obb_transform[:3, 0]
        self.y_axis = self.obj_obb_transform[:3, 1]
        self.z_axis = self.obj_obb_transform[:3, 2]

        self.obj_obb_vertices = np.zeros((8, 3))

        self.obj_obb_vertices[0] = self.obj_obb_center + self.x_range * self.x_axis + self.y_range * self.y_axis + self.z_range * self.z_axis
        self.obj_obb_vertices[1] = self.obj_obb_center - self.x_range * self.x_axis + self.y_range * self.y_axis + self.z_range * self.z_axis
        self.obj_obb_vertices[2] = self.obj_obb_center + self.x_range * self.x_axis - self.y_range * self.y_axis + self.z_range * self.z_axis
        self.obj_obb_vertices[3] = self.obj_obb_center - self.x_range * self.x_axis - self.y_range * self.y_axis + self.z_range * self.z_axis
        self.obj_obb_vertices[4] = self.obj_obb_center + self.x_range * self.x_axis + self.y_range * self.y_axis - self.z_range * self.z_axis
        self.obj_obb_vertices[5] = self.obj_obb_center - self.x_range * self.x_axis + self.y_range * self.y_axis - self.z_range * self.z_axis
        self.obj_obb_vertices[6] = self.obj_obb_center + self.x_range * self.x_axis - self.y_range * self.y_axis - self.z_range * self.z_axis
        self.obj_obb_vertices[7] = self.obj_obb_center - self.x_range * self.x_axis - self.y_range * self.y_axis - self.z_range * self.z_axis
        
        # TODO: There is a problem when the OBB projection is parallel to the x and y axis. 
        # There are two planes of the OBB parallel to the xy-plane in the world frame
        # max and min x, y, z coordinates index
        self.max_v_idx = self.obj_obb_vertices.argmax(axis=0)
        self.min_v_idx = self.obj_obb_vertices.argmin(axis=0)

        # self.v_idx_argsort = np.argsort(self.obj_obb_vertices, axis=0)

        # Compute the axis of the obb w.r.t. the world frame
        # We cannot directly use the axis from the transform. Because we do not know which axes are 
        # within the horizontal plane.
        x_axis = self.obj_obb_vertices[self.min_v_idx[1]] - self.obj_obb_vertices[self.max_v_idx[0]]
        y_axis = self.obj_obb_vertices[self.max_v_idx[1]] - self.obj_obb_vertices[self.max_v_idx[0]]


        x_axis = self.obj_obb_vertices[self.min_v_idx[1]] - self.obj_obb_vertices[self.max_v_idx[0]]
        y_axis = self.obj_obb_vertices[self.max_v_idx[1]] - self.obj_obb_vertices[self.max_v_idx[0]]
        
        x_axis = x_axis[0:2]
        y_axis = y_axis[0:2]
        
        # x and y range of the obb (anchored at the max_x vertex of obb)
        self.obj_obb_x_range = np.linalg.norm(x_axis)
        self.obj_obb_y_range = np.linalg.norm(y_axis)

        # x- and y-axis of the obb in the world frame (anchored at the max_x vertex of obb)
        self.obj_obb_x_axis = x_axis / self.obj_obb_x_range
        self.obj_obb_y_axis = y_axis / self.obj_obb_y_range

        self.obb_anchor_xy = self.obj_obb_vertices[self.max_v_idx[0]][0:2]


    def load_sphere(self):
        """ 
        Load sphere before simulation.
        Compute the position of each sphere (Get the list of the position of the sphere).
        Make sure that pybullet has already been connected before calling this function.             
        """

        sphere = p.loadURDF(self.sphere_urdf, basePosition=[0, 0, -1])

        if "jelly_bean" in self.sphere_urdf:
            p.resetBasePositionAndOrientation(sphere, posObj=[0, 0, -1], ornObj=p.getQuaternionFromEuler([0, np.pi/2, 0]))

        p.changeDynamics(sphere, -1, restitution=self.sphere_restitution, 
                lateralFriction=self.sphere_lateralfriction,
                spinningFriction=0.5,
                rollingFriction=0.5)

        sphere_aabb = p.getAABB(sphere)
        p.removeBody(sphere)

        self.sphere_x_range = sphere_aabb[1][0] - sphere_aabb[0][0]
        self.sphere_y_range = sphere_aabb[1][1] - sphere_aabb[0][1] 
        
        # Dropping from 1cm above the object
        self.sphere_drop_z = self.obj_curr_aabb[1][2] + 0.01

        # Sphere range projected in obj obb x-axis
        self.sphere_x_range_obj_obb = self.sphere_x_range * abs(self.obj_obb_x_axis[0]) + self.sphere_y_range * abs(self.obj_obb_x_axis[1])
        self.x_sphere_num = int(self.obj_obb_x_range / (self.sphere_x_range_obj_obb)) + 1

        # Sphere range projected in obj obb y-axis
        self.sphere_y_range_obj_obb = self.sphere_x_range * abs(self.obj_obb_y_axis[0]) + self.sphere_y_range * abs(self.obj_obb_y_axis[1])
        self.y_sphere_num = int(self.obj_obb_y_range / (self.sphere_y_range_obj_obb)) + 1
        
        self.drop_sphere_num = self.x_sphere_num * self.y_sphere_num

        if self.drop_sphere_num > self.sphere_num_max:
            self.sphere_dist_scale = math.sqrt(self.drop_sphere_num / self.sphere_num_max)
            self.x_sphere_num = int(self.obj_obb_x_range / (self.sphere_dist_scale * self.sphere_x_range_obj_obb)) + 1
            self.y_sphere_num = int(self.obj_obb_y_range / (self.sphere_dist_scale * self.sphere_y_range_obj_obb)) + 1
            self.drop_sphere_num = self.x_sphere_num * self.y_sphere_num

        # Set up sphere
        for i in range(self.drop_sphere_num):
            sphere = p.loadURDF(self.sphere_urdf)
            p.changeDynamics(sphere, -1, restitution=self.sphere_restitution, 
                    lateralFriction=self.sphere_lateralfriction,
                    spinningFriction=0.5,
                    rollingFriction=0.5)

            self.sphere_id.append(sphere)

        # Reset the sphere_drop_pos
        self.sphere_drop_pos = []

        # Reset sphere position
        for i in range(self.x_sphere_num):
            for j in range(self.y_sphere_num):
                sphere_idx = i * self.y_sphere_num + j
                sphere_start_pos = np.zeros(3)
                sphere_start_pos[-1] = self.sphere_drop_z
                sphere_start_pos[:2] = self.obb_anchor_xy + \
                                       i * self.sphere_x_range_obj_obb * self.obj_obb_x_axis * self.sphere_dist_scale + \
                                       j * self.sphere_y_range_obj_obb * self.obj_obb_y_axis * self.sphere_dist_scale

                self.sphere_drop_pos.append(sphere_start_pos)
                p.resetBasePositionAndOrientation(self.sphere_id[sphere_idx], 
                        posObj=sphere_start_pos, 
                        ornObj=self.sphere_drop_orn)  


    def checkincup(self, obj_curr_aabb):
        """ Get how many spheres are in the cup and the original position of the sphere
            Also, the drop pos of the in sphere will be updated to self.sphere_in_drop_pos
        """

        # Clear the sphere_in_drop_pos memory
        self.sphere_in_drop_pos = []

        # The center and range of the aabb. All scales are magnified by 100 times.
        obj_aabb_center = np.array([(obj_curr_aabb[0][i] + obj_curr_aabb[1][i])/2 for i in range(3)]) * 100
        obj_aabb_half_range = np.array([abs(obj_curr_aabb[0][i] - obj_curr_aabb[1][i]) for i in range(3)]) * 100 / 2

        for i in range(self.drop_sphere_num):
            sphere_pos, _ = p.getBasePositionAndOrientation(self.sphere_id[i])
            sphere_pos = np.array(sphere_pos) * 100
            
            distance_to_center = np.absolute(sphere_pos - obj_aabb_center)
            in_box = distance_to_center < obj_aabb_half_range

            if np.all(in_box):
                # print("x: %.2f" % x, "y: %.2f" % y, "z: %.2f" % z)
                self.sphere_in_drop_pos.append(np.copy(self.sphere_drop_pos[i]))

        return len(self.sphere_in_drop_pos)


    def get_containability(self):
        """ 
        Test the pouring of sphere and check how many sphere remains after pouring. 
        """
        # Load sphere
        self.load_sphere()
        import ipdb; ipdb.set_trace()

        ########################### Drop Sphere Into ############################
        # 2.0: Shake Objects
        pivot = [0, 0, 1]

        for i in range(self.simulation_iteration):
            p.stepSimulation()

            if self.check_process:
                time.sleep(1. / 240.)

            if i == int(1 * self.simulation_iteration / 10):
                # Check the number of sphere in the bbox before moving the sphere away
                sphere_in_num = self.checkincup(self.obj_curr_aabb)            

            # 2.0: Shake Objects
            if i > int(1 * self.simulation_iteration / 10) and i <= int(5 * self.simulation_iteration / 10):
                orn = p.getQuaternionFromEuler([math.pi/60 * math.sin(math.pi * 2 * (i - int(1 * self.simulation_iteration / 10)) / int(4 * self.simulation_iteration / 10)), 0, 0])
                p.changeConstraint(self.constraint_Id, pivot, jointChildFrameOrientation=orn, maxForce=50)
            elif i > int(5 * self.simulation_iteration / 10) and i <= int(9 * self.simulation_iteration / 10):
                orn = p.getQuaternionFromEuler([0, math.pi/60 * math.sin(math.pi * 2 * (i - int(5 * self.simulation_iteration / 10)) / int(4 * self.simulation_iteration / 10)), 0])
                p.changeConstraint(self.constraint_Id, pivot, jointChildFrameOrientation=orn, maxForce=50)

            # 3.0: Horizontal Acceleration
            elif i > int(9 * self.simulation_iteration / 10) and i <= int(9.25 * self.simulation_iteration / 10):
                p.setGravity(0.5, 0.0, -9.81)
            elif i > int(9.25 * self.simulation_iteration / 10) and i <= int(9.5 * self.simulation_iteration / 10):
                p.setGravity(-0.5, 0.0, -9.81)
            elif i > int(9.5 * self.simulation_iteration / 10) and i <= int(9.75 * self.simulation_iteration / 10):
                p.setGravity(0.0, 0.5, -9.81)
            elif i > int(9.75 * self.simulation_iteration / 10) and i <= int(10 * self.simulation_iteration / 10):
                p.setGravity(0.0, -0.5, -9.81)

        ########## 2.0 Version of checking sphere ##########
        # Check the x, y, z coordinate of the sphere w.r.t to the x, y, z coordinate of the cup
        sphere_in_box_num = self.checkincup(self.obj_curr_aabb)

        # Calculate how many percentage of the spheres are in the cup
        sphere_num_percentage = sphere_in_box_num / self.drop_sphere_num

        if sphere_num_percentage > self.sphere_in_percentage_threshold:
            print("#####################################")
            print("THIS IS A CONTAINER! The sphere in percentage is: {}".format(sphere_num_percentage))
            print("#####################################")
            self.containability = True

        else:
            print("/////////////////////////////////////")
            print("THIS IS NOT A CONTAINER!The sphere in percentage is: {}".format(sphere_num_percentage))
            print("/////////////////////////////////////")
            self.containability = False

        return self.containability, sphere_num_percentage
    

    def find_drop_center(self):
        """ Find the center to drop bean """
        if len(self.sphere_in_drop_pos) < 1:
            print("No sphere remains in the object after drops!")
            return None
        else:
            self.sphere_in_drop_pos = np.array(self.sphere_in_drop_pos)
            x = np.mean(self.sphere_in_drop_pos[:, 0])
            y = np.mean(self.sphere_in_drop_pos[:, 1])
            z = self.sphere_drop_z
            drop_center_curr_frame = np.array([x, y, z])
            
            # Debug
            # p.loadURDF(self.sphere_urdf, basePosition=[x, y, z])

            # Original frame 2 current frame
            R = np.reshape(np.array(p.getMatrixFromQuaternion(self.obj_zero_orn)), (3, 3))
            t = self.obj_zero_pos

            # drop_center_original_frame = np.linalg.inv(R) @ (drop_center_curr_frame - np.array(t)).T
            drop_center_original_frame = np.dot(np.linalg.inv(R), (drop_center_curr_frame - np.array(t)).T)

            return drop_center_original_frame

    def disconnect_p(self):
        p.disconnect()

if __name__ == "__main__":
    obj_urdf = "/home/hongtao/Dropbox/ICRA2021/data/training_set/Container/"
    obj_vhacd_mesh = "/home/hongtao/Dropbox/ICRA2021/data/training_set/Container"
    C = Containability(obj_urdf, obj_vhacd_mesh, check_process=True)
    C.get_containability()