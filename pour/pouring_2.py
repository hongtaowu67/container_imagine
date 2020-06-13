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

class BottlePour(object):
    def __init__(self, bottle_urdf, content_urdf, obj_urdf, pour_pos, indent_num=1, content_num=40,
                obj_zero_pos=[0, 0, 0], obj_zero_orn=[0, 0, 0], 
                check_process=False, mp4_dir=None, object_name=None):
        """
        Args:
        -- bottle_obj: the urdf of the pouring bottle
        -- obj_urdf: the urdf of the object being poured
        -- pour_pos (np array in [x y z]): the position of the pour point
        -- indent_num: the number of indent in a single pouring angle
        -- obj_zero_pos (list): x y z of the object initial position
        -- obj_zero_orn (list): Euler angle (roll pitch yaw) of the object initial orientation
        """
        super(BottlePour, self).__init__()
        
        if check_process:
            self.pysical_client = p.connect(p.GUI)
        else:
            self.physical_client = p.connect(p.DIRECT)

        # Save mp4 video
        if mp4_dir is not None and check_process:
            self.save_mp4_dir = mp4_dir
            self.object_name = object_name
            mp4_file_name = self.object_name + "_pour.mp4"
            mp4_file_path = os.path.join(self.save_mp4_dir, mp4_file_name)
            p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, mp4_file_path)
        
        # p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
        # Reset debug camera postion
        p.resetDebugVisualizerCamera(0.7, 0, -40, [-0.05, -0.1, 1])

        self.pour_simulation_iteration = 500
        self.wait_simultaion_iteration = 100
        self.check_process = check_process
        p.setGravity(0, 0, -9.81)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.plane_id = p.loadURDF("plane.urdf")

        # Bottle
        self.bottle_urdf = bottle_urdf
        self.pour_angle = 2 * np.pi / 5
        self.pour_pos_nominal = pour_pos + np.array(obj_zero_pos) # need to add the position of the object
        self.pour_num = 8 # Number of pouring within [0, 2*pi)

        # Content
        self.content_urdf = content_urdf
        self.content_num = content_num
        self.content_id_list = []
        content_aabb_id = p.loadURDF(self.content_urdf)
        self.content_aabb = p.getAABB(content_aabb_id)
        p.removeBody(content_aabb_id)
        self.content_restitution = 0.0
        self.content_lateralfriction = 0.005
        for i in range(self.content_num):
            content_id = p.loadURDF(content_urdf)
            self.content_id_list.append(content_id)

        # Object
        self.obj_id = p.loadURDF(obj_urdf, basePosition=obj_zero_pos)
        self.obj_raise = 0.1
        self.obj_zero_orn = p.getQuaternionFromEuler(obj_zero_orn)
        self.obj_zero_pos = obj_zero_pos
        self.obj_aabb = p.getAABB(self.obj_id)
        
        # Create constraint on the cup to fix its position
        p.changeDynamics(self.obj_id, -1, mass=1)
        self.constraint_Id = p.createConstraint(self.obj_id, -1, -1, -1, p.JOINT_FIXED, jointAxis=[0, 0, 0],
                parentFramePosition=[0, 0, 0], childFramePosition=self.obj_zero_pos)

        # Pour
        self.indent_num = indent_num
        self.obj_x_range = self.obj_aabb[1][0] - self.obj_aabb[0][0]
        self.obj_y_range = self.obj_aabb[1][1] - self.obj_aabb[0][1]
        self.obj_digonal_len = math.sqrt(self.obj_x_range * self.obj_x_range + self.obj_y_range * self.obj_y_range)
        self.indent_len = self.obj_digonal_len / (3 * self.indent_num) # half the length of the diagonal
        self.canonical_bottle_angle = np.pi/4
        self.pivot_pos_list = []


    def bottle_pour(self, indent=0.01):
        """
        Rotate the bottle about the pour_pos.
        """
        self.spill_list = []

        for k in range(self.pour_num):
            planar_angle = k / self.pour_num * 2 * np.pi
            spill_angle_list = []
            pivot_pos_angle_list = []

            for j in range(self.indent_num):

                # Pour position for different angle. Indent is included for the offset from the nominal pour pos.
                self.pour_pos = np.zeros(3)
                self.pour_pos[0] = self.pour_pos_nominal[0] - (indent + j * self.indent_len) * np.cos(planar_angle)
                self.pour_pos[1] = self.pour_pos_nominal[1] - (indent + j * self.indent_len) * np.sin(planar_angle)
                self.pour_pos[2] = self.pour_pos_nominal[2] #+ 0.001

                # Load Bottle Bottle
                self.bottle_id = p.loadURDF(self.bottle_urdf)
                self.bottle_pos = np.array([0.0, 0.0, 0.0])
                p.changeDynamics(self.bottle_id, -1, mass=1)
                self.bottle_aabb = p.getAABB(self.bottle_id)

                # For debug
                # p.loadURDF(self.content_urdf, basePosition=self.pour_pos)

                bottle_half_length = self.bottle_aabb[1][0] #- 0.003
                pour_pos_offset = - bottle_half_length * np.array([np.cos(planar_angle), np.sin(planar_angle)])
                self.bottle_pos[0] = self.pour_pos[0] + pour_pos_offset[0]
                self.bottle_pos[1] = self.pour_pos[1] + pour_pos_offset[1]
                self.bottle_pos[2] = self.pour_pos[2] + (-self.bottle_aabb[0][2] - 0.014) # offset for the tip of the bottle

                p.resetBasePositionAndOrientation(self.bottle_id,
                                                posObj=self.bottle_pos,
                                                ornObj=p.getQuaternionFromEuler([0, 0, planar_angle]))

                # parentFramePosition: the joint frame pos in the object frame
                # childFramePosition: the joint frame pos in the world frame if the child frame is set to be -1 (base)
                # parentFrameOrientation: the joint frame orn in the object frame
                # childFrameOrientation: the joint frame orn in the world frame if the child frame is set to be -1 (base)
                bottle_constraint_Id = p.createConstraint(self.bottle_id, -1, -1, -1, p.JOINT_FIXED, jointAxis=[0, 0, 0],
                                                        parentFramePosition=[bottle_half_length, 0.0, self.pour_pos[2]-self.bottle_pos[2]], 
                                                        childFramePosition=self.pour_pos,
                                                        parentFrameOrientation=p.getQuaternionFromEuler([0, 0, 0]),
                                                        childFrameOrientation=p.getQuaternionFromEuler([0, 0, planar_angle]))

                self.set_content(planar_angle)
                
                pivot = self.pour_pos

                for i in range(self.pour_simulation_iteration):
                    p.stepSimulation()

                    if self.check_process:
                        time.sleep(1. / 240.)           

                    orn = p.getQuaternionFromEuler([0, 2*np.pi/5 * math.sin(math.pi * 2 * (i) / int(4 * self.pour_simulation_iteration)), planar_angle])
                    p.changeConstraint(bottle_constraint_Id, pivot, jointChildFrameOrientation=orn, maxForce=50)

                for i in range(self.wait_simultaion_iteration):
                    p.stepSimulation()

                    if self.check_process:
                        time.sleep(1. / 240.)      
                
                spill = self.check_spillage()
                p.removeBody(self.bottle_id)

                spill_angle_list.append(spill)
                pivot_pos_angle_list.append(pivot)

            self.spill_list.append(spill_angle_list)
            self.pivot_pos_list.append(pivot_pos_angle_list)

        return self.spill_list

    def set_content(self, planar_angle):
        """
        Set contents to the position (in the bottle).
        """

        # Contents are loaded at the middle between the bottle coneter and the bottle bottom
        content_pos = self.bottle_pos
        x_range = np.abs(self.bottle_aabb[0][0]) - np.abs(self.content_aabb[1][0] - self.content_aabb[0][0]) * 2
        y_range = np.abs(self.bottle_aabb[1][1] - self.bottle_aabb[0][1]) - np.abs(self.content_aabb[1][1] - self.content_aabb[0][1]) * 4
        z_range = np.abs(self.bottle_aabb[1][2] - self.bottle_aabb[0][2]) - np.abs(self.content_aabb[1][2] - self.content_aabb[0][2]) * 4

        x_num_range = np.floor(x_range / np.abs(self.content_aabb[1][0] - self.content_aabb[0][0]))
        y_num_range = np.floor(y_range / np.abs(self.content_aabb[1][1] - self.content_aabb[0][1]))
        z_num_range = np.floor(z_range / np.abs(self.content_aabb[1][2] - self.content_aabb[0][2]))

        for i in range(self.content_num):
            content_id = self.content_id_list[i]
            x_offset = np.random.randint(1, x_num_range+1) / x_num_range * x_range
            y_offset = np.random.randint(1, y_num_range+1) / y_num_range * y_range + self.bottle_aabb[0][1] - self.content_aabb[0][1] * 2
            z_offset = np.random.randint(1, z_num_range+1) / z_num_range * z_range + self.bottle_aabb[0][2] - self.content_aabb[0][2] * 2

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


    def check_spillage(self):
        """
        Check every content and see if it is on the ground.
        """
        
        spill_num = 0
        for i in range(self.content_num):
            content_pos, content_orn = p.getBasePositionAndOrientation(self.content_id_list[i]) # (content_pos, content_quaternion)
            content_z = content_pos[2]
            spill_num += content_z < self.obj_aabb[0][2]
        
        return spill_num

    def best_pour_pos_orn(self):
        """
        Supporting only the canonical bottle angle at the moment
        
        Return:
            - pivot_pos: (3, ) numpy array, the pouring position
            - bottle_angle: angle for pouring
        """
        # Spill angle list corresponds to the canonical bottle angle
        canonical_spill_angle_list = np.array(self.spill_list[1])
        spill_min_idx = np.argmin(canonical_spill_angle_list)

        if canonical_spill_angle_list[spill_min_idx] == 0:
            pivot_pos = self.pivot_pos_list[1][spill_min_idx] - self.obj_zero_pos
            bottle_angle = self.canonical_bottle_angle
        
        return pivot_pos, bottle_angle
        

    def disconnect_p(self):
        p.disconnect()



if __name__ == "__main__":
    obj_urdf = "/home/hongtao/Dropbox/ICRA2021/data/training_set/Container/Blue_Cup/Blue_Cup_mesh_0.urdf"
    bottle_urdf = "/home/hongtao/Dropbox/ICRA2021/data/general/bottle/JuiceBottle_GeoCenter.urdf"
    content_urdf = "/home/hongtao/Dropbox/ICRA2021/data/general/m&m.urdf"
    pour_pos = np.array([-0.08763265508145837, -0.26190104459071484, 0.19611099708080304])
    mp4_dir = "/home/hongtao/Desktop"
    obj_name = "Blue_Cup"
    BP = BottlePour(bottle_urdf, content_urdf, obj_urdf, pour_pos, obj_zero_pos=[0, 0, 1], indent_num=3,
                    check_process=False)#, mp4_dir=mp4_dir, object_name=obj_name)

    spill_list = BP.bottle_pour()
    print "spill list"
    print spill_list
    print "pivot_pos_list"
    print BP.pivot_pos_list

    pivot_pos, bottle_angle = BP.best_pour_pos_orn()
    print "pivot position: ", pivot_pos
    print "bottle angle: ", bottle_angle
    BP.disconnect_p()


    