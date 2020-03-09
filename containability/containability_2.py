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
"""
from __future__ import division

import numpy as np
import pybullet as p
import pybullet_data
import time
import os
import math


sphere_urdf = "/home/hongtao/Dropbox/ICRA2021/data/general/sphere_mini.urdf"

class Containability(object):
    def __init__(self, obj_urdf, obj_zero_pos=[0, 0, 0], obj_zero_orn=[0, 0, 0], 
                 check_process=False, record_process=False, mp4_dir=None, object_name=None):
        """
        Args:
        - obj_zero_orn: the start orientation of the object in Euler Angle
        - check_process: if set True, the process will be real time
        - record_process: if set True, the process will be recorded and saved as mp4 file
        """
        super(Containability, self).__init__()
        # Hyperparameter
        self.sphere_num = 225  # sphere number needs to be a 
        self.sphere_in_percentage_threshold = 0.08
        self.sphere_in_percentage = 0.0

        # Restitution
        self.sphere_restitution = 0.0
        self.object_restitution = 0.0
        self.plane_restitution = 0.0

        self.obj_urdf = obj_urdf
        self.containability = False

        # Simulation Parameter
        self.simulation_iteration = 1500
        self.check_process = check_process
        self.record_process = record_process

        # Sphere Information
        self.sphere_id = []
        self.sphere_drop_orn = p.getQuaternionFromEuler([0, 0, 0])
        self.sphere_drop_pos = []
        self.sphere_in_drop_pos = []
        self.sphere_drop_z = 0
        self.sphere_lateralfriction=0.0

        # Set the world
        physicsClient = p.connect(p.GUI)
        p.setGravity(0, 0, -10)

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

        # Raise the object up
        if self.obj_curr_aabb[0][2] <= 0.1:
            p.resetBasePositionAndOrientation(self.obj_id, 
                    posObj=(0, 0, -self.obj_curr_aabb[0][2]+0.1),
                    ornObj=self.obj_zero_orn)
            self.obj_curr_aabb = p.getAABB(self.obj_id)

        # Create constraint on the cup to fix its position
        p.changeDynamics(self.obj_id, -1, mass=1)
        self.constraint_Id = p.createConstraint(self.obj_id, -1, -1, -1, p.JOINT_FIXED, jointAxis=[0, 0, 0],
                parentFramePosition=[0, 0, 0], childFramePosition=self.obj_zero_pos)
        


    def load_sphere(self):
        """ Load sphere before simulation. 
            Make sure that pybullet has already been connected before calling this function.             
        """

        # Set up sphere
        for i in range(self.sphere_num):
            sphere = p.loadURDF(sphere_urdf)
            p.changeDynamics(sphere, -1, restitution=self.sphere_restitution, 
                    lateralFriction=self.sphere_lateralfriction,
                    spinningFriction=0.5,
                    rollingFriction=0.5)

            self.sphere_id.append(sphere)

    
    def reset_sphere_drop(self, obj_curr_aabb):
        """ Get the sphere drop position around the obj_curr_aabb. This function is called after the set_sphere"""

        obj_aabb_xy_center = [(obj_curr_aabb[0][i] + obj_curr_aabb[1][i])/2 for i in range(2)]
        obj_aabb_xy_range = [abs(obj_curr_aabb[0][i] - obj_curr_aabb[1][i]) for i in range(2)]
        
        # Dropping from 1cm above the object
        self.sphere_drop_z = obj_curr_aabb[1][2] + 0.01

        sphere_per_length = math.sqrt(self.sphere_num/(obj_aabb_xy_range[0] * obj_aabb_xy_range[1]))

        x_sphere_num = int(sphere_per_length * obj_aabb_xy_range[0])
        y_sphere_num = int(sphere_per_length * obj_aabb_xy_range[1])

        sphere_drop_x = np.linspace(-obj_aabb_xy_range[0]/2, obj_aabb_xy_range[0]/2, x_sphere_num) + obj_aabb_xy_center[0]
        sphere_drop_y = np.linspace(-obj_aabb_xy_range[1]/2, obj_aabb_xy_range[1]/2, y_sphere_num) + obj_aabb_xy_center[1]

        # Reset the sphere_drop_pos
        self.sphere_drop_pos = []

        # Reset sphere position
        for i in range(self.sphere_num):
            y_idx = int(i / x_sphere_num)
            x_idx = i % x_sphere_num

            try:
                sphere_start_pos = (sphere_drop_x[x_idx], sphere_drop_y[y_idx], self.sphere_drop_z)
            except:
                # For those out of range, position the sphere in a random xy in a higher layer
                sphere_start_pos = (obj_aabb_xy_center[0] + np.random.random() * obj_aabb_xy_range[0]/2, \
                                    obj_aabb_xy_center[1] + np.random.random() * obj_aabb_xy_range[1]/2, \
                                    self.sphere_drop_z + 0.05)

            self.sphere_drop_pos.append(sphere_start_pos)
            p.resetBasePositionAndOrientation(self.sphere_id[i], 
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

        for i in range(self.sphere_num):
            sphere_pos, _ = p.getBasePositionAndOrientation(self.sphere_id[i])
            sphere_pos = np.array(sphere_pos) * 100
            
            distance_to_center = np.absolute(sphere_pos - obj_aabb_center)
            in_box = distance_to_center < obj_aabb_half_range

            if np.all(in_box):
                # print("x: %.2f" % x, "y: %.2f" % y, "z: %.2f" % z)
                self.sphere_in_drop_pos.append(np.copy(self.sphere_drop_pos[i]))

        return len(self.sphere_in_drop_pos)


    def get_containability(self):
        """ Test the pouring of sphere and check how many sphere remains after pouring. """
        # Load sphere
        self.load_sphere()
        # Reset sphere position
        self.reset_sphere_drop(self.obj_curr_aabb)

        ####### Get OBB of the object ########
        ####### Get CoM of the object ########

        ########################### Drop Sphere Into ############################
        force = 1

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
                orn = p.getQuaternionFromEuler([math.pi/40 * math.sin(math.pi * 2 * (i - int(1 * self.simulation_iteration / 10)) / int(4 * self.simulation_iteration / 10)), 0, 0])
                p.changeConstraint(self.constraint_Id, pivot, jointChildFrameOrientation=orn, maxForce=50)
            elif i > int(5 * self.simulation_iteration / 10) and i <= int(9 * self.simulation_iteration / 10):
                orn = p.getQuaternionFromEuler([0, math.pi/40 * math.sin(math.pi * 2 * (i - int(5 * self.simulation_iteration / 10)) / int(4 * self.simulation_iteration / 10)), 0])
                p.changeConstraint(self.constraint_Id, pivot, jointChildFrameOrientation=orn, maxForce=50)

            # 3.0: Horizontal Acceleration
            elif i > int(9 * self.simulation_iteration / 10) and i <= int(9.25 * self.simulation_iteration / 10):
                p.setGravity(0.5, 0.0, -10)
            elif i > int(9.25 * self.simulation_iteration / 10) and i <= int(9.5 * self.simulation_iteration / 10):
                p.setGravity(-0.5, 0.0, -10)
            elif i > int(9.5 * self.simulation_iteration / 10) and i <= int(9.75 * self.simulation_iteration / 10):
                p.setGravity(0.0, 0.5, -10)
            elif i > int(9.75 * self.simulation_iteration / 10) and i <= int(10 * self.simulation_iteration / 10):
                p.setGravity(0.0, -0.5, -10)

        ########## 2.0 Version of checking sphere ##########
        # Check the x, y, z coordinate of the sphere w.r.t to the x, y, z coordinate of the cup
        sphere_in_box_num = self.checkincup(self.obj_curr_aabb)

        # Calculate how many percentage of the spheres are in the cup
        sphere_num_percentage = sphere_in_box_num / self.sphere_num

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
            # p.loadURDF(sphere_urdf, basePosition=[x, y, z])

            # Original frame 2 current frame
            R = np.reshape(np.array(p.getMatrixFromQuaternion(self.obj_zero_orn)), (3, 3))
            t = self.obj_zero_pos

            # drop_center_original_frame = np.linalg.inv(R) @ (drop_center_curr_frame - np.array(t)).T
            drop_center_original_frame = np.dot(np.linalg.inv(R), (drop_center_curr_frame - np.array(t)).T)

            return drop_center_original_frame


    def disconnet(self):
        p.disconnect()
        

# # Test
# if __name__ == "__main__":

#     # Object information
#     model_root_dir = "/home/hongtao/Dropbox/ICRA2021/data"

#     object_subdir = "GripperTest1_24view"

#     object_name = object_subdir + "_mesh_debug_0"
#     obj_urdf = os.path.join(model_root_dir, object_subdir, object_name + '.urdf')

#     mp4_dir = os.path.join(model_root_dir, object_subdir)
#     print('URDF: ', obj_urdf)

#     C = Containability(obj_urdf, obj_zero_pos=[0, 0, 1], obj_zero_orn=[0, 0, 0], 
#             check_process=False, mp4_dir=None, object_name=object_name)

#     containable_affordance, sphere_in_percentage = C.get_containability()

#     if containable_affordance:
#         drop_spot = C.find_drop_center()
#         print("Dropping at: {}".format(drop_spot))
#     else:
#         drop_spot = [np.nan, np.nan, np.nan]
    
#     obj_containability_filename = object_subdir + '_py2.txt'
#     obj_containability_file = os.path.join(model_root_dir, object_subdir, obj_containability_filename)
#     with open(obj_containability_file, 'w') as writefile:
#         write_line=[]
#         obj_info = "Name: " + object_subdir + '\n'
#         write_line.append(obj_info)
#         containability = "Containability: " + str(containable_affordance) + '\n'
#         write_line.append(containability)
#         sphere_percentage_line = "Sphere in percentage: " + str(sphere_in_percentage) + '\n'
#         write_line.append(sphere_percentage_line)
#         dropspot = "Drop Spot: " + str(list(drop_spot)) + '\n'
#         write_line.append(dropspot)
#         today = time.strftime("%Y-%m-%d-%H-%M", time.localtime())
#         time = "Time: " + today + '\n'
#         write_line.append(time)
#         writefile.writelines(write_line)

#     C.disconnet()