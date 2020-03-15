import os
import time

import numpy as np
import cv2
import rospy

from robot import Robot
from reconstruction.ros_camera_tsdf_fusion import ROSCameraTSDFFusion
from utils import angle2rotm, make_rigid_transformation


class PickAndPour:
    def __init__ (self, acc, vel):
        self.acc = acc
        self.vel = vel

        self.robot = Robot(acc=self.acc, vel=self.vel, gripper_on=True)
        
        # Configurations to pick the bottle
        self.pick_bottle_joints = [
            # Low
            (2.006856679916382, -1.3167336622821253, 1.6932334899902344, -1.9662039915667933, -1.5629408995257776, -1.0797608534442347),
            # High
            (2.007300853729248, -1.3760116736041468, 1.5031867027282715, -1.7167752424823206, -1.5620291868792933, -1.0819900671588343),
            # # Intermediate 1
            # (2.007300853729248, -1.3760116736041468, 1.5031867027282715, -1.7167752424823206, -1.5620291868792933, -1.0819900671588343 - np.pi),
            # Intermediate 2
            (2.007300853729248, -1.3760116736041468, 1.5031867027282715, -1.5576642195331019, -1.5620291868792933, -1.0819900671588343 - np.pi),
            # Intermediate 3
            (2.008380889892578, -1.3056567350970667, 0.8938207626342773, -1.0186932722674769, -1.5632646719561976, -4.228302303944723),

        ]
        
        # start pour
        # Unit: angle
        self.pre_pour_orn = [3.02589579,  0.0, -0.16839265]
        self.pre_pour_orn_angle = np.linalg.norm(np.array(self.pre_pour_orn))
        self.pre_pour_orn_axis = np.array(self.pre_pour_orn) / self.pre_pour_orn_angle

        # Rotation matrix
        self.pre_pour_orn_mat = angle2rotm(self.pre_pour_orn_angle, self.pre_pour_orn_axis)

        # axis for offset
        self.pre_pour_x_axis = self.pre_pour_orn_mat[:, 0]
        self.pre_pour_y_axis = self.pre_pour_orn_mat[:, 1]
        self.pre_gripper_offset = -0.08
        self.pre_gripper_ee_offset = self.pre_gripper_offset * self.pre_pour_x_axis * np.sin(np.pi/4) - self.pre_gripper_offset * self.pre_pour_y_axis * np.cos(np.pi/4)
        self.pre_gripper_ee_offset_z = 0.22

        # mid pour
        self.mid_pour_orn = [2.48899947, 0.0, -0.72901107]
        self.mid_pour_orn_angle = np.linalg.norm(np.array(self.mid_pour_orn))
        self.mid_pour_orn_axis = np.array(self.mid_pour_orn) / self.mid_pour_orn_angle
        # Rotation matrix
        self.mid_pour_orn_mat = angle2rotm(self.mid_pour_orn_angle, self.mid_pour_orn_axis)
        self.mid_pour_z_axis = self.mid_pour_orn_mat[:, 2]
        self.mid_gripper_offset = -0.08
        self.mid_gripper_ee_offset = self.mid_pour_z_axis * self.mid_gripper_offset
        self.mid_gripper_ee_offset_z = 0.20

        # end pour
        self.end_pour_orn = [2.03187381,  0.        , -1.04386125]
        self.end_pour_orn_angle = np.linalg.norm(np.array(self.end_pour_orn))
        self.end_pour_orn_axis = np.array(self.end_pour_orn) / self.end_pour_orn_angle

        self.end_pour_orn_mat = angle2rotm(self.end_pour_orn_angle, self.end_pour_orn_axis)
        self.end_pour_z_axis = self.end_pour_orn_mat[:, 2]
        self.end_gripper_offset = -0.14
        self.end_gripper_ee_offset = self.end_gripper_offset * self.end_pour_z_axis
        self.end_gripper_ee_offset_z = 0.16

        # # axis for rotation
        # self.rotate_axis = self.pre_pour_x_axis * np.sin(np.pi/4) + self.pre_pour_y_axis * np.cos(np.pi/4)
        # self.rotate_axis = self.rotate_axis[:3]
        # self.rotate_angle = np.pi/2

        # # rigid body transformation of the pour point in the tool frame
        # self.pour_point_t = np.array([-self.pre_gripper_ee_offset[0] - 0.01, -self.pre_gripper_ee_offset[1] - 0.01, -0.2])
        # self.pour_point_R = np.zeros((3, 3))
        # self.pour_point_R[2, 2] = -1
        # self.pour_point_R[:, 1] = self.rotate_axis
        # self.pour_point_R[:, 0] = - self.pre_pour_x_axis * np.sin(np.pi/4) + self.pre_pour_y_axis * np.cos(np.pi/4)
        # self.pour_point_g = make_rigid_transformation(self.pour_point_t, self.pour_point_R)

        

    def pick(self):
        # Move to the top of the bottle
        self.robot.move_to_joint(self.pick_bottle_joints[1])

        # Move the robot down
        self.robot.move_to_joint(self.pick_bottle_joints[0])
        
        # Close the gripper
        self.robot.close_gripper()

        # Move the robot up
        self.robot.move_to_joint(self.pick_bottle_joints[1])

        self.robot.move_to_joint(self.pick_bottle_joints[2])
        self.robot.move_to_joint(self.pick_bottle_joints[3])

    

    def pour(self, pour_pos):
        pre_pour_pos_x = pour_pos[0] + self.pre_gripper_ee_offset[0]
        pre_pour_pos_y = pour_pos[1] + self.pre_gripper_ee_offset[1]
        pre_pour_pos_z = pour_pos[2] + self.pre_gripper_ee_offset_z
        pre_pour_pos = [pre_pour_pos_x, pre_pour_pos_y, pre_pour_pos_z]

        self.robot.move_to(pre_pour_pos, self.pre_pour_orn, acc=0.5, vel=0.5)

        # time.sleep(1)

        # mid_pour_pos_x = pour_pos[0] + self.mid_gripper_ee_offset[0]
        # mid_pour_pos_y = pour_pos[1] + self.mid_gripper_ee_offset[1]
        # mid_pour_pos_z = pour_pos[2] + self.mid_gripper_ee_offset_z
        # mid_pour_pos = [mid_pour_pos_x, mid_pour_pos_y, mid_pour_pos_z]

        # self.robot.move_to(mid_pour_pos, self.mid_pour_orn)

        end_pour_pos_x = pour_pos[0] + self.end_gripper_ee_offset[0]
        end_pour_pos_y = pour_pos[1] + self.end_gripper_ee_offset[1]
        end_pour_pos_z = pour_pos[2] + self.end_gripper_ee_offset_z
        end_pour_pos = [end_pour_pos_x, end_pour_pos_y, end_pour_pos_z]

        self.robot.move_to(end_pour_pos, self.end_pour_orn, acc=0.1, vel=0.1)

        current_config = self.robot.get_config()
        pour_config = current_config

        # Shake the bottle
        rotate_angle = np.pi/180 * 3
        shake_config = pour_config
        shake_config_last = shake_config[-1]
        for i in range(6):
            if i % 2 == 0:
                shake_config[-1] = shake_config_last + rotate_angle
                self.robot.move_to_joint(shake_config, acc=5.0, vel=5.0)
            else:
                shake_config[-1] = shake_config_last - rotate_angle
                self.robot.move_to_joint(shake_config, acc=5.0, vel=5.0)
        
        shake_config[-1] = shake_config_last
        self.robot.move_to_joint(shake_config, acc=5.0, vel=5.0)

        self.robot.disconnect()

    def disconnect(self):
        self.robot.disconnect()


# if __name__ == "__main__":
#     PP = PickAndPour(0.3, 0.3)
#     PP.pick()
#     pour_pos = [-0.09652546464564028, -0.2380368459759542, 0.20159800696372998]
#     PP.pour(pour_pos)

# >>> r2 = utils.angle2rotm(-np.pi*2/5, np.array([np.sin(np.pi/4), -np.cos(np.pi/4), 0]))
# >>> axis = utils.rotm2angle(np.dot(r2, r1)[:3, :3])
# >>> axis = np.array(axis)
# >>> axis = axis[1:] * axis[0]




