# This script imagines pouring from a container onto the object.
# Find the best pouring spot and angle for pouring into a specific object.
# Instead of aligning with the world x- and y-axis, the cup openning align with the principal
# axis of the footprint of the contents remaining in the cup after the drop.

# Author: Hongtao Wu
# Institution: Johns Hopkins University
# Date: June 25, 2020

from __future__ import division, print_function

import numpy as np
import pybullet as p
import pybullet_data
import os
import time
import math
import trimesh

from sklearn.decomposition import PCA
from utils import isRotm, rotm2angle


class CupPour(object):
    """
    Class for pouring imagination with a cup
    """
    def __init__(self,
                 cup_urdf,
                 content_urdf,
                 obj_urdf,
                 pour_pos,
                 content_in_list_se2,
                 indent_num=1,
                 content_num=40,
                 obj_zero_pos=[0, 0, 0],
                 obj_zero_orn=[0, 0, 0],
                 check_process=False,
                 mp4_dir=None,
                 object_name=None):
        """
        @type  cup_urdf: string
        @param cup_urdf: the urdf file of the pouring cup
        @type  content_urdf: string
        @param content_urdf: the urdf file of the content
        @type  obj_urdf: string
        @param obj_urdf: the urdf of the object being poured
        @type  pour_pos: numpy.ndarray
        @param pour_pos ([x y z]): the position of the pour point
        @type  content_in_list: numpy.ndarray, shape is (n, 2)
        @param content_in_list: se2 drop postions of contents remained in the object in the containability imagiantion
        @type  indent_num: int
        @param indent_num: the number of indent in a single pouring angle
        @type  obj_zero_pos: list
        @param obj_zero_pos: [x y z] of the object initial position
        @type  obj_zero_orn: list
        @param obj_zero_orn (list): Euler angle (roll pitch yaw) of the object initial orientation
        @type  check_process: bool
        @param check_process: whether to slow down the simulation to check
        @type  mp4_dir: string
        @param mp4_dir: if set as not None, it will save a mp4 file of the imagination process in the directory
        @type  object_name: string
        @param object_name: name of the object. Note it is not the data name. A data is about a single take of
            the scene. A data can have many object in the scene
        """
        super(CupPour, self).__init__()

        if check_process:
            self.physical_client = p.connect(p.GUI)
        else:
            self.physical_client = p.connect(p.DIRECT)

        # Save mp4 video
        if mp4_dir is not None and check_process:
            self.save_mp4_dir = mp4_dir
            self.object_name = object_name
            mp4_file_name = self.object_name + "_pour.mp4"
            mp4_file_path = os.path.join(self.save_mp4_dir, mp4_file_name)
            p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, mp4_file_path)

        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        # Reset debug camera postion
        p.resetDebugVisualizerCamera(0.7, 0, -40, [-0.05, -0.1, 1])

        self.pour_simulation_iteration = 500
        self.wait_simultaion_iteration = 100
        self.check_process = check_process
        p.setGravity(0, 0, -9.81)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.plane_id = p.loadURDF("plane.urdf")

        # cup
        self.cup_urdf = cup_urdf
        self.pour_angle = np.pi / 4
        self.pour_pos_nominal = pour_pos + np.array(
            obj_zero_pos)  # need to add the position of the object
        self.pour_num = 8  # Number of pouring within [0, 2*pi)

        # Content
        self.content_urdf = content_urdf
        self.content_num = content_num
        self.content_id_list = []
        content_aabb_id = p.loadURDF(self.content_urdf)
        self.content_aabb = p.getAABB(content_aabb_id)
        p.removeBody(content_aabb_id)
        self.content_restitution = 0.0
        self.content_lateralfriction = 0.2
        self.content_spinningfriction = 0.1
        self.content_rollingfriction = 0.1
        for i in range(self.content_num):
            content_id = p.loadURDF(content_urdf)
            self.content_id_list.append(content_id)
            p.changeDynamics(content_id,
                             -1,
                             restitution=self.content_restitution,
                             lateralFriction=self.content_lateralfriction,
                             spinningFriction=self.content_spinningfriction,
                             rollingFriction=self.content_rollingfriction)

        # Object
        self.obj_id = p.loadURDF(obj_urdf, basePosition=obj_zero_pos)
        self.obj_raise = 0.1
        self.obj_zero_orn = p.getQuaternionFromEuler(obj_zero_orn)
        self.obj_zero_pos = obj_zero_pos
        self.obj_aabb = p.getAABB(self.obj_id)

        # Create constraint on the cup to fix its position
        p.changeDynamics(self.obj_id, -1, mass=1)
        self.constraint_Id = p.createConstraint(
            self.obj_id,
            -1,
            -1,
            -1,
            p.JOINT_FIXED,
            jointAxis=[0, 0, 0],
            parentFramePosition=[0, 0, 0],
            childFramePosition=self.obj_zero_pos)

        # Pour
        self.indent_num = indent_num
        self.obj_x_range = self.obj_aabb[1][0] - self.obj_aabb[0][0]
        self.obj_y_range = self.obj_aabb[1][1] - self.obj_aabb[0][1]
        self.obj_digonal_len = math.sqrt(self.obj_x_range * self.obj_x_range +
                                         self.obj_y_range * self.obj_y_range)
        self.indent_len = self.obj_digonal_len / (
            3 * self.indent_num)  # half the length of the diagonal
        self.content_in_list_se2 = content_in_list_se2
        # List of spillage at different pouring pos and cup angle
        self.spill_list = []
        # List of pivot position at diffrent pouring pos and cup angle
        self.pivot_pos_list = []
        # The SO(2) angle of principal axis with largest variance
        self.content_large_var_angle = self.get_PCA_orn()

    def cup_pour(self, indent=0.01):
        """
        Rotate the cup to pour. The pouring position is close to the pour_pos.
        But since the contents have horizontal velocity when exiting the opening,
        we need to indent the cup backwards when pouring.
        
        @type  indent: float
        @param indent: the indent distance from the pour_pos.
        """
        for k in range(self.pour_num):
            planar_angle = self.content_large_var_angle + k / self.pour_num * 2 * np.pi
            spill_angle_list = []
            pivot_pos_angle_list = []

            for j in range(self.indent_num):

                # Pour position for different angle. Indent is included for x the offset from the nominal pour pos.
                self.pour_pos = np.zeros(3)
                self.pour_pos[0] = self.pour_pos_nominal[0] - (
                    indent + j * self.indent_len) * np.cos(planar_angle)
                self.pour_pos[1] = self.pour_pos_nominal[1] - (
                    indent + j * self.indent_len) * np.sin(planar_angle)
                self.pour_pos[2] = self.pour_pos_nominal[2]

                # Load cup
                self.cup_id = p.loadURDF(self.cup_urdf)
                self.cup_pos = np.array([0.0, 0.0, 0.0])
                p.changeDynamics(self.cup_id, -1, mass=1)
                self.cup_aabb = p.getAABB(self.cup_id)

                cup_half_length = self.cup_aabb[1][0] - 0.006
                pour_pos_offset = -cup_half_length * np.array(
                    [np.cos(planar_angle),
                     np.sin(planar_angle)])
                self.cup_pos[0] = self.pour_pos[0] + pour_pos_offset[0]
                self.cup_pos[1] = self.pour_pos[1] + pour_pos_offset[1]
                self.cup_pos[2] = self.pour_pos[2] + (
                    -self.cup_aabb[0][2] - 0.01
                )  # offset for the tip of the cup

                self.cup_initial_orn = [0, 0, planar_angle]

                p.resetBasePositionAndOrientation(
                    self.cup_id,
                    posObj=self.cup_pos,
                    ornObj=p.getQuaternionFromEuler(self.cup_initial_orn))

                # parentFramePosition: the joint frame pos in the object frame
                # childFramePosition: the joint frame pos in the world frame if the child frame is set to be -1 (base)
                # parentFrameOrientation: the joint frame orn in the object frame
                # childFrameOrientation: the joint frame orn in the world frame if the child frame is set to be -1 (base)
                cup_constraint_Id = p.createConstraint(
                    self.cup_id,
                    -1,
                    -1,
                    -1,
                    p.JOINT_FIXED,
                    jointAxis=[0, 0, 0],
                    parentFramePosition=[
                        cup_half_length, 0.0,
                        self.pour_pos[2] - self.cup_pos[2]
                    ],
                    childFramePosition=self.pour_pos,
                    parentFrameOrientation=p.getQuaternionFromEuler([0, 0, 0]),
                    childFrameOrientation=p.getQuaternionFromEuler(
                        self.cup_initial_orn))

                self.set_content(planar_angle)

                pivot = self.pour_pos

                for i in range(self.pour_simulation_iteration):
                    p.stepSimulation()

                    if self.check_process:
                        time.sleep(1. / 240.)

                    # Need to compensate for the np.pi/30 in the cup zero orientation
                    orn = p.getQuaternionFromEuler([
                        0, (0.028575317269292654 + self.pour_angle) *
                        math.sin(math.pi * 2 * i /
                                 int(4 * self.pour_simulation_iteration)),
                        planar_angle
                    ])
                    p.changeConstraint(cup_constraint_Id,
                                       pivot,
                                       jointChildFrameOrientation=orn,
                                       maxForce=50)

                for i in range(self.wait_simultaion_iteration):
                    p.stepSimulation()

                    if self.check_process:
                        time.sleep(1. / 240.)

                spill = self.check_spillage()

                p.removeBody(self.cup_id)

                spill_angle_list.append(spill)
                pivot_pos_angle_list.append(pivot)

            self.spill_list.append(spill_angle_list)
            self.pivot_pos_list.append(pivot_pos_angle_list)

        return self.spill_list

    def cup_pour_at(self, imagined_pour_pos, imagined_cup_angle):
        """
        Visualize the imagined configuration of the cup before pouring.

        @type  imagined_pour_pos: numpy.ndarray
        @param imagined_pour_pos: the pouring position from the pouring imagination
        @type  imagined_cup_angle: float
        @param imagined_cup_angle: SO2 rotation angle in the horizontal plane
        """
        p.resetDebugVisualizerCamera(0.7, 45, -30, [-0.09, -0.1, 1])
        planar_angle = imagined_cup_angle

        # Pour position for different angle. Indent is included for x the offset from the nominal pour pos.
        self.pour_pos = np.zeros(3)
        self.pour_pos[0] = imagined_pour_pos[0]
        self.pour_pos[1] = imagined_pour_pos[1]
        self.pour_pos[2] = imagined_pour_pos[2] + self.obj_zero_pos[2]

        # Load cup
        self.cup_id = p.loadURDF(self.cup_urdf)
        self.cup_pos = np.array([0.0, 0.0, 0.0])
        p.changeDynamics(self.cup_id, -1, mass=1)
        self.cup_aabb = p.getAABB(self.cup_id)

        # # For debug
        # p.loadURDF(self.content_urdf, basePosition=self.pour_pos, useFixedBase=True)

        cup_half_length = self.cup_aabb[1][0] - 0.006
        pour_pos_offset = -cup_half_length * np.array(
            [np.cos(planar_angle), np.sin(planar_angle)])
        self.cup_pos[0] = self.pour_pos[0] + pour_pos_offset[0]
        self.cup_pos[1] = self.pour_pos[1] + pour_pos_offset[1]
        self.cup_pos[2] = self.pour_pos[2] + (
            -self.cup_aabb[0][2] - 0.01)  # offset for the tip of the cup

        self.cup_initial_orn = [0, 0, planar_angle]

        p.resetBasePositionAndOrientation(self.cup_id,
                                          posObj=self.cup_pos,
                                          ornObj=p.getQuaternionFromEuler(
                                              self.cup_initial_orn))

        # parentFramePosition: the joint frame pos in the object frame
        # childFramePosition: the joint frame pos in the world frame if the child frame is set to be -1 (base)
        # parentFrameOrientation: the joint frame orn in the object frame
        # childFrameOrientation: the joint frame orn in the world frame if the child frame is set to be -1 (base)
        cup_constraint_Id = p.createConstraint(
            self.cup_id,
            -1,
            -1,
            -1,
            p.JOINT_FIXED,
            jointAxis=[0, 0, 0],
            parentFramePosition=[
                cup_half_length, 0.0, self.pour_pos[2] - self.cup_pos[2]
            ],
            childFramePosition=self.pour_pos,
            parentFrameOrientation=p.getQuaternionFromEuler([0, 0, 0]),
            childFrameOrientation=p.getQuaternionFromEuler(
                self.cup_initial_orn))

        self.set_content(planar_angle)

        import ipdb
        ipdb.set_trace()

    def set_content(self, planar_angle):
        """
        Set contents to the position (in the cup).

        @type  planar_angle: float
        @param planar_angle: 
        """

        # Contents are loaded at the middle between the cup coneter and the cup bottom
        content_pos = self.cup_pos
        x_range = np.abs(self.cup_aabb[1][0] - self.cup_aabb[0][0]
                         ) / 2 - 0.008  # offset for the cup base
        y_range = 0.06 / np.sqrt(2) - np.abs(
            self.content_aabb[1][1] - self.content_aabb[0][1]
        )  # Consider patterning the content in a cylindrical with 0.06 diameter
        z_range = 0.06 / np.sqrt(2) - np.abs(
            self.content_aabb[1][2] - self.content_aabb[0][2]
        )  # Consider patterning the content in a cylindrical with 0.06 diameter

        x_num_range = np.floor(
            x_range /
            np.abs(self.content_aabb[1][0] - self.content_aabb[0][0]))
        y_num_range = np.floor(
            y_range /
            np.abs(self.content_aabb[1][1] - self.content_aabb[0][1]))
        z_num_range = np.floor(
            z_range /
            np.abs(self.content_aabb[1][2] - self.content_aabb[0][2]))

        cup_rotm = p.getMatrixFromQuaternion(
            p.getQuaternionFromEuler([0, -np.pi / 30, planar_angle]))
        cup_rotm = np.array(cup_rotm).reshape(3, 3)

        for i in range(self.content_num):
            content_id = self.content_id_list[i]
            x_offset = np.random.randint(
                1, x_num_range + 1) / x_num_range * x_range
            y_offset = np.random.randint(
                1, y_num_range + 1) / y_num_range * y_range
            z_offset = np.random.randint(
                1, z_num_range + 1) / z_num_range * z_range

            pos = np.array([
                x_offset - x_range / 2, y_offset - y_range / 2,
                z_offset - z_range / 2
            ])
            pos_angle = np.dot(cup_rotm, pos)

            p.resetBasePositionAndOrientation(
                content_id,
                posObj=(content_pos[0] + pos_angle[0],
                        content_pos[1] + pos_angle[1],
                        content_pos[2] + pos_angle[2]),
                ornObj=self.obj_zero_orn)

        # Let the contents to drop
        for i in range(100):
            p.stepSimulation()

    def check_spillage(self):
        """
        Check every content and see if it is on the ground.
        """

        spill_num = 0
        for i in range(self.content_num):
            content_pos, content_orn = p.getBasePositionAndOrientation(
                self.content_id_list[i])  # (content_pos, content_quaternion)
            content_z = content_pos[2]
            spill_num += content_z < self.obj_aabb[0][2]

        return spill_num

    def best_pour_pos_orn(self):
        """
        Calculate the best pouring position and cup angle.
        1. Select the pouring position and cup angle based on the spillage.
        2. Select the cup angle close to the principal axis with largest variance
        
        @rtype  pivot_pos: (3, ) numpy.ndarray
        @return pivot_pos: the pouring position
        @rtype  cup_angle: float
        @return cup_angle: angle for pouring
        """

        # Pick the best orn and pos by selecting the pouring orn and pos
        # with minimum spillage among all the pouring orn and pos

        spill_list = np.array(self.spill_list)
        spill_list_angle_sum = np.sum(spill_list, axis=1)
        min_spillage_sum_angle = spill_list_angle_sum.min()
        cup_angle_idx_list = np.where(
            spill_list_angle_sum == min_spillage_sum_angle)

        min_spill_num = self.content_num
        min_spill_angle_idx = None
        min_spill_angle_pos_idx = None

        for cup_angle_idx in cup_angle_idx_list[0]:
            spill_angle_list = spill_list[cup_angle_idx]
            spill_angle_pos_min_idx = np.argmin(spill_angle_list)

            if spill_angle_list[spill_angle_pos_min_idx] < min_spill_num:
                min_spill_num = spill_angle_list[spill_angle_pos_min_idx]
                min_spill_angle_idx = cup_angle_idx
                min_spill_angle_pos_idx = spill_angle_pos_min_idx
            elif spill_angle_list[spill_angle_pos_min_idx] == min_spill_num:
                # Pick the one closest to the center
                if spill_angle_pos_min_idx < min_spill_angle_pos_idx:
                    min_spill_angle_idx = cup_angle_idx
                    min_spill_angle_pos_idx = spill_angle_pos_min_idx
                elif spill_angle_pos_min_idx == min_spill_angle_pos_idx:
                    # Pick the one closest to the principal axis
                    if min(cup_angle_idx,
                           abs(cup_angle_idx - self.pour_num / 2)) < min(
                               min_spill_angle_idx,
                               abs(min_spill_angle_idx - self.pour_num / 2)):
                        min_spill_angle_idx = cup_angle_idx
                        min_spill_angle_pos_idx = spill_angle_pos_min_idx

        print("min_spill_angle_idx: ", min_spill_angle_idx)
        print("min_spill_angle_pos_idx: ", min_spill_angle_pos_idx)

        pivot_pos = self.pivot_pos_list[min_spill_angle_idx][
            min_spill_angle_pos_idx] - self.obj_zero_pos
        cup_angle = self.content_large_var_angle + min_spill_angle_idx * np.pi / 4

        if cup_angle > np.pi:
            cup_angle -= 2 * np.pi

        print("Best pour cup angle: ", cup_angle)
        print("Best pour pos: ", pivot_pos)

        return pivot_pos, cup_angle

    def get_PCA_orn(self):
        """
        Compute the PCA of a list of content dropped in the container.

        @rtype conent_large_var_angle: float
        @return content_large_var_angle: SO(2) orientation corresponding to the largest variance
        """
        pca = PCA(n_components=2)
        pca.fit(self.content_in_list_se2)

        large_var_axis = pca.components_[0]
        content_large_var_angle = np.arctan2(large_var_axis[1],
                                             large_var_axis[0])

        return content_large_var_angle

    def disconnect_p(self):
        """
        Disconnect pybullet
        """
        p.disconnect()