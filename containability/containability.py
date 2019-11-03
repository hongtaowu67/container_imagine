#! /usr/bin/env python

"""
This script is written to check whether an object is able to contain sphere.
No physics about the cup is reasoned. This scrip only test the containability
This is written for cup imagination project.
@author: Hongtao Wu 
Nov 02, 2019
"""

import numpy as np
import pybullet as p
import pybullet_data
import time
import os
import math

sphere_urdf = "/home/hongtao/Dropbox/spirit-dictionary/dataset/general_object/sphere_mini.urdf"

class Containability(object):
    def __init__(self, obj_urdf, obj_zero_pos=[0, 0, 0], obj_zero_orn=[0, 0, 0], 
                 check_process=False, record_process=False):
        """
        Args:
        - obj_zero_orn: the start orientation of the object in Euler Angle
        - check_process: if set True, the process will be real time
        - record_process: if set True, the process will be recorded and saved as mp4 file
        """
        super(Containability, self).__init__()
        # Hyperparameter
        self.sphere_num = 225  # sphere number needs to be a 
        self.sphere_in_percentage_threshold = 0.2
        self.sphere_in_percentage = 0.0

        # Restitution
        self.sphere_restitution = 0.0
        self.object_restitution = 0.0
        self.plane_restitution = 0.0

        self.obj_urdf = obj_urdf
        self.containability = False

        # Simulation Parameter
        self.simulation_iteration = 1000
        self.save_mp4_dir = "/home/hongtao/Dropbox/spirit-dictionary/mp4"
        self.check_process = check_process
        self.record_process = record_process

        # Sphere Information
        self.sphere_id = []
        self.sphere_drop_orn = p.getQuaternionFromEuler([0, 0, 0])
        self.sphere_drop_pos = []
        self.sphere_in_drop_pos = []

        # Set the world
        physicsClient = p.connect(p.GUI)
        p.setGravity(0, 0, -10)

        # restitutionId = p.addUserDebugParameter("restitution", 0.5)
        # restitution = p.readUserDebugParameter(restitutionId)

        # restitutionVelocityThresholdId = p.addUserDebugParameter("res. vel. threshold", 0, 3, 0.2)
        # restitutionVelocityThreshold = p.readUserDebugParameter(restitutionVelocityThresholdId)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        # p.setPhysicsEngineParameter(restitutionVelocityThreshold=restitutionVelocityThreshold)

        # Save mp4 video
        if self.record_process:
            today = time.strftime("%Y-%m-%d-%H-%M-%S", time.localtime())
            mp4_file_name = today + "_retain_sphere_test.mp4"
            mp4_file_path = os.path.join(self.save_mp4_dir, mp4_file_name)
            p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, mp4_file_path)

        # Load plane
        self.plane_id = p.loadURDF("plane.urdf")
        p.changeDynamics(self.plane_id, -1, restitution=self.plane_restitution)

        # Load object
        obj_zero_pos = obj_zero_pos
        obj_zero_orn = p.getQuaternionFromEuler(obj_zero_orn)
        self.obj_id = p.loadURDF(self.obj_urdf, basePosition=obj_zero_pos, baseOrientation=obj_zero_orn, useFixedBase=True)
        p.changeDynamics(self.obj_id, -1, restitution=self.object_restitution)

        # Get the bounding box of the cup
        self.obj_curr_aabb = p.getAABB(self.obj_id)

        # Raise the object up
        if self.obj_curr_aabb[0][2] <= 0.1:
            p.resetBasePositionAndOrientation(self.obj_id, 
                    posObj=(0, 0, -self.obj_curr_aabb[0][2]+0.1),
                    ornObj=obj_zero_orn)
            self.obj_curr_aabb = p.getAABB(self.obj_id)

        # Reset debug camera postion
        p.resetDebugVisualizerCamera(2.0, 75, -50, [0, 0, 0.5])

        # Create constraint on the cup to fix its position
        constarin_Id = p.createConstraint(self.obj_id, -1, -1, -1, p.JOINT_FIXED, jointAxis=[0, 0, 0],
                parentFramePosition=[0, 0, 0], childFramePosition=obj_zero_pos,
                parentFrameOrientation=p.getQuaternionFromEuler([0, 0, 0]),
                childFrameOrientation=obj_zero_orn)


    def load_sphere(self):
        """ Load sphere before simulation. 
            Make sure that pybullet has already been connected before calling this function.             
        """

        # Set up sphere
        for i in range(self.sphere_num):
            sphere = p.loadURDF(sphere_urdf)
            p.changeDynamics(sphere, -1, restitution=self.sphere_restitution)

            self.sphere_id.append(sphere)

    
    def reset_sphere_drop(self, obj_curr_aabb):
        """ Get the sphere drop position around the obj_curr_aabb. This function is called after the set_sphere"""

        obj_aabb_xy_center = [(obj_curr_aabb[0][i] + obj_curr_aabb[1][i])/2 for i in range(2)]
        obj_aabb_xy_range = [abs(obj_curr_aabb[0][i] - obj_curr_aabb[1][i]) for i in range(2)]
        
        # Dropping from 1cm above the object
        sphere_drop_z = obj_curr_aabb[1][2] + 0.01

        sphere_per_length = math.sqrt(self.sphere_num/(obj_aabb_xy_range[0] * obj_aabb_xy_range[1]))

        x_sphere_num = math.floor(sphere_per_length * obj_aabb_xy_range[0])
        y_sphere_num = math.floor(sphere_per_length * obj_aabb_xy_range[1])

        sphere_drop_x = np.linspace(-obj_aabb_xy_range[0]/2, obj_aabb_xy_range[0]/2, x_sphere_num) + obj_aabb_xy_center[0]
        sphere_drop_y = np.linspace(-obj_aabb_xy_range[1]/2, obj_aabb_xy_range[1]/2, y_sphere_num) + obj_aabb_xy_center[1]

        # Reset the sphere_drop_pos
        self.sphere_drop_pos = []

        # Reset sphere position
        for i in range(self.sphere_num):
            y_idx = math.floor(i / x_sphere_num)
            x_idx = i % x_sphere_num

            try:
                sphere_start_pos = (sphere_drop_x[x_idx], sphere_drop_y[y_idx], sphere_drop_z)
            except:
                # For those out of range, position the sphere in a random xy in a higher layer
                sphere_start_pos = (obj_aabb_xy_center[0] + np.random.random() * obj_aabb_xy_range[0]/2, \
                                    obj_aabb_xy_center[1] + np.random.random() * obj_aabb_xy_range[1]/2, \
                                    sphere_drop_z + 0.05)
            
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

        ########################### Drop Sphere Into ############################

        for i in range(self.simulation_iteration):
            p.stepSimulation()

            if self.check_process:
                time.sleep(1. / 240.)

            if i == int(1 * self.simulation_iteration / 5):
                # Check the number of sphere in the bbox before moving the sphere away
                sphere_in_num = self.checkincup(self.obj_curr_aabb)

            if i > int(1 * self.simulation_iteration / 5) and i < int(1 * self.simulation_iteration / 2):
                horizontal_field_strength = 5 + 5 * math.sin(math.pi / 2 * (i - int(self.simulation_iteration / 5)) / int(3 * self.simulation_iteration / 10))
                p.setGravity(horizontal_field_strength, horizontal_field_strength, -10)

            elif i >= int(1 * self.simulation_iteration / 2) and i < int(3 * self.simulation_iteration / 5):
                p.setGravity(10, 10, -10)

            elif i >= int(3 * self.simulation_iteration / 5) and i < int(9 * self.simulation_iteration / 10):
                horizontal_field_strength = -5 - 5 * math.sin(math.pi / 2 * (i - int(3 * self.simulation_iteration / 5)) / int(3 * self.simulation_iteration / 10))
                p.setGravity(horizontal_field_strength, horizontal_field_strength, -10)

            elif i >= int(9 * self.simulation_iteration / 10):
                p.setGravity(-10, -10, -10)


        ########## 2.0 Version of checking sphere ##########
        # Check the x, y, z coordinate of the sphere w.r.t to the x, y, z coordinate of the cup
        sphere_in_box_num = self.checkincup(self.obj_curr_aabb)
        print("Number of sphere in box: ", sphere_in_box_num)

        # Calculate how many percentage of the spheres are in the cup
        sphere_num_percentage = sphere_in_box_num / self.sphere_num

        # Disconnect the 
        p.disconnect()

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
        
        return self.containability


# Test
if __name__ == "__main__":

    # Object information
    obj_name = "cup_0003"
    obj_urdf = "/home/hongtao/Dropbox/spirit-dictionary/dataset/cup_imagine/cup_physical_properties/" + \
    str(obj_name) + "/" + str(obj_name) + ".urdf"

    C = Containability(obj_urdf, obj_zero_pos=[0, 0, 1], obj_zero_orn=[math.pi/2, 0, 0])

    containable_affordance = C.get_containability()



#     def get_pour_into_able(self, obj_stable_orn_valid, obj_stable_pos_valid, obj_sphere_in_pos_valid):
#         # Circle Idea: calculate the center of all the valid sphere position

#         physicsClient = p.connect(p.GUI)
#         p.setAdditionalSearchPath(pybullet_data.getDataPath())

#         restitutionId = p.addUserDebugParameter("restitution", 0.5)
#         restitution = p.readUserDebugParameter(restitutionId)

#         restitutionVelocityThresholdId = p.addUserDebugParameter("res. vel. threshold", 0, 3, 0.2)
#         restitutionVelocityThreshold = p.readUserDebugParameter(restitutionVelocityThresholdId)

#         p.setPhysicsEngineParameter(restitutionVelocityThreshold=restitutionVelocityThreshold)

#         # Reset camera postion
#         p.resetDebugVisualizerCamera(1.2, 75, -50, [0, 0, 0])

#         # Load the world
#         plane_id = p.loadURDF("plane.urdf")
#         p.changeDynamics(plane_id, -1, restitution=restitution)

#         # Load the cup
#         obj_start_pos = obj_stable_pos_valid
#         obj_start_orn = p.getQuaternionFromEuler(obj_stable_orn_valid)
#         obj_id = p.loadURDF(self.obj_urdf, obj_start_pos, obj_start_orn)
#         p.changeDynamics(obj_id, -1, restitution=restitution)

#         # Load the sphere
#         ps = PourCupTest(self.pour_sphere_num)

#         # Save mp4 video
#         if self.record_process:
#             today = time.strftime("%Y-%m-%d-%H-%M-%S", time.localtime())
#             mp4_file_name = today + "_pour_sphere_test.mp4"
#             mp4_file_path = os.path.join(self.save_mp4_dir, mp4_file_name)
#             p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, mp4_file_path)

#         # For now, we assume that there is only one valid pose.
#         # We are picking the pose which can contain the most sphere.
#         obj_final_pos = np.copy(obj_stable_pos_valid)
#         obj_final_pos[-1] = obj_final_pos[-1] + 0.2
#         obj_final_orn = obj_stable_orn_valid
#         obj_final_orn_qua = p.getQuaternionFromEuler(obj_final_orn)
#         # if obj_final_orn_qua[0] > 0:
#         #     obj_final_orn_qua = (-obj_final_orn_qua[0], -obj_final_orn_qua[1], -obj_final_orn_qua[2], -obj_final_orn_qua[3])
#         # print("Cup pour orientation in quaternion: ", obj_final_orn_qua)
#         p.resetBasePositionAndOrientation(obj_id, obj_final_pos, obj_final_orn_qua)

#         # Create constraint on the cup
#         constarin_Id = p.createConstraint(obj_id, -1, -1, -1, p.JOINT_FIXED, jointAxis=[0, 0, 0],
#                                           parentFramePosition=[0, 0, 0], childFramePosition=obj_final_pos,
#                                           parentFrameOrientation=p.getQuaternionFromEuler([0, 0, 0]),
#                                           childFrameOrientation=obj_final_orn_qua)

#         sphere_circle_center = np.sum(np.array(obj_sphere_in_pos_valid), axis=0) / len(obj_sphere_in_pos_valid)
#         sphere_circle_center_xy = sphere_circle_center[:2]

#         cup_curr_bbox = p.getAABB(obj_id)
#         print("Cup Bounding Box: ", cup_curr_bbox)

#         # We are rotating about the y axis. Pour step forward is proportional to the x range of the cup
#         pour_step_forward = abs(cup_curr_bbox[0][0] - cup_curr_bbox[1][0]) / 10
#         sphere_circle_center_xy[0] = sphere_circle_center_xy[0] - pour_step_forward * (5 - 1) / 2

#         sphere_xy = np.array(obj_sphere_in_pos_valid)[:, :2]
#         sphere_circle_radius_list = np.linalg.norm(sphere_xy - sphere_circle_center_xy, axis=1)
#         sphere_circle_radius_max = np.max(sphere_circle_radius_list)
#         print("sphere_circle_center_pos: ", sphere_circle_center_xy)
#         print("sphere_circle_max: ", sphere_circle_radius_max)

#         print("Start checking pouring into!")
#         for pour_test_idx in range(5):
#             sphere_circle_center_xy[0] += pour_step_forward

#             ps.reset_cup_sphere_position(sphere_circle_center_xy, cup_curr_bbox)

#             p.setGravity(0, 0, -10)

#             for k in range(int(self.simulation_iteration / 4)):
#                 p.stepSimulation()
#                 if self.check_process:
#                     time.sleep(1. / 240.)
#                 time.sleep(1. / 240.)

#             p.setGravity(0, 0, 0)
#             pour_in_cup_percentage = ps.checkincup(cup_curr_bbox) / self.pour_sphere_num
#             if pour_in_cup_percentage > self.pour_sphere_in_number_percentage_threshold:
#                 print("#########################################")
#                 print("The object is pourable and containable!")
#                 print("The pour_in_cup_percentage is: ", pour_in_cup_percentage)
#                 print("#########################################")
#                 break

#         p.disconnect()


# # Sphere for testing cup functionality
# class SphereCupTest(object):
#     def __init__(self, sphere_num, sphere_restitution):
#         super(SphereCupTest, self).__init__()
#         self.sphere_num = sphere_num
#         self.sphere_id = []
#         self.sphere_start_orn = [0, 0, 0]
#         for i in range(self.sphere_num):
#             sphere = p.loadURDF(sphere_urdf, [0, 0, 0], p.getQuaternionFromEuler(self.sphere_start_orn))
#             p.changeDynamics(sphere, -1, restitution=sphere_restitution)
#             self.sphere_id.append(sphere)
#         self.sphere_start_pos = np.zeros((sphere_num, 3))

#     def resetspherePosOrn(self, sphere_start_xy, cup_curr_bbox):
#         x_range_value = abs(cup_curr_bbox[0][0] - cup_curr_bbox[1][0])
#         y_range_value = abs(cup_curr_bbox[0][1] - cup_curr_bbox[1][1])
#         z_range = np.array([cup_curr_bbox[0][2], cup_curr_bbox[1][2]])

#         ten_value = int(math.sqrt(self.sphere_num))

#         for i in range(self.sphere_num):

#             # # Spheres are placed randomly within the bounding box in a uniform random probability
#             # sphere_start_pos = [x_range_value * (0.5 - np.random.random()) + sphere_start_xy[0], y_range_value * (0.5 - np.random.random()) + sphere_start_xy[1], z_range[1] + 0.01]
#             # self.sphere_start_pos[i, :] = np.array(sphere_start_pos)
#             # p.resetBasePositionAndOrientation(self.sphere_id[i], sphere_start_pos, p.getQuaternionFromEuler(self.sphere_start_orn))

#             # Sphere are placed in order within the bounding box
#             ten_idx = math.floor(i / ten_value)
#             one_idx = i % ten_value
#             sphere_x = sphere_start_xy[0] - x_range_value * 0.5 + one_idx * x_range_value / ten_value
#             sphere_y = sphere_start_xy[1] - y_range_value * 0.5 + ten_idx * y_range_value / ten_value
#             sphere_start_pos = [sphere_x, sphere_y, z_range[1] + 0.01]
#             self.sphere_start_pos[i, :] = np.array(sphere_start_pos)
#             p.resetBasePositionAndOrientation(self.sphere_id[i], sphere_start_pos, p.getQuaternionFromEuler(self.sphere_start_orn))

#     def resetspherePosOrnSphere(self, center_xy, center_radius, cupCurrBbox):
#         z_range = np.array([cupCurrBbox[0][2], cupCurrBbox[1][2]])

#         for i in range(self.sphere_num):
#             theta = np.random.random() * math.pi * 2
#             radius = np.random.random() * center_radius
#             x = center_xy[0] + radius * math.cos(theta)
#             y = center_xy[1] + radius * math.sin(theta)
#             sphereStartPos = [x, y, z_range[1] + 0.01]
#             self.sphere_start_pos[i, :] = np.array(sphereStartPos)
#             p.resetBasePositionAndOrientation(self.sphere_id[i], sphereStartPos, p.getQuaternionFromEuler(self.sphere_start_orn))

#     def getspherePosOrn(self, sphere_idx):
#         return p.getBasePositionAndOrientation(self.sphere_id[sphere_idx])

#     def checkincup(self, cupCurrBbox):
#         sphere_start_pos_in_list = []
#         x_range = np.array([cupCurrBbox[0][0], cupCurrBbox[1][0]]) * 100
#         y_range = np.array([cupCurrBbox[0][1], cupCurrBbox[1][1]]) * 100
#         z_range_value = abs(cupCurrBbox[0][2] - cupCurrBbox[1][2])
#         z_range = np.array([cupCurrBbox[0][2], cupCurrBbox[1][2] - z_range_value * 0.1]) * 100  # The ball shall be within the height of 0.9 of the cup height
#         inbox_num = 0
#         for j in range(self.sphere_num):
#             spherePos, _ = p.getBasePositionAndOrientation(self.sphere_id[j])
#             x = spherePos[0] * 100
#             y = spherePos[1] * 100
#             z = spherePos[2] * 100

#             if (x-x_range[0]) * (x-x_range[1]) <= 0 and (y-y_range[0]) * (y-y_range[1]) <= 0 and (z-z_range[0]) * (z-z_range[1]) <=0:
#                 # print("x: %.2f" % x, "y: %.2f" % y, "z: %.2f" % z)
#                 inbox_num += 1
#                 sphere_start_pos_in_list.append(np.copy(self.sphere_start_pos[j]))
#         return inbox_num, sphere_start_pos_in_list


# # Quaternion multiplication
# def quaternion_multiply(quaternion1, quaternion0):
#     a1, b1, c1, d1 = quaternion0
#     a2, b2, c2, d2 = quaternion1
#     return np.array([
#                     a1*a2 - b1*b2 - c1*c2 - d1*d2,
#                     a1*b2 + a2*b1 + c1*d2 - c2*d1,
#                     a1*c2 + a2*c1 + b2*d1 - b1*d2,
#                     a1*d2 + a2*d1 + b1*c2 - b2*c1], dtype=np.float64)

