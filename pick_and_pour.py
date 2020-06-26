import os
import time

import numpy as np
import cv2
import rospy

from robot import Robot
from reconstruction.ros_camera_tsdf_fusion import ROSCameraTSDFFusion
from utils import angle2rotm


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
            (2.007300853729248, -1.3760116736041468, 1.5031867027282715, -1.7167752424823206, -1.5620291868792933, -1.0819900671588343)
        ]
        
        # Unit: angle
        # self.pre_pour_orn = [0.08087661589319343, -2.4383649694242595, 0.8312275202696641]
        self.pre_pour_orn = [0.07419675171714485, -2.1324334762895134, 1.0622946386306489]
        self.pre_pour_orn_angle = np.linalg.norm(np.array(self.pre_pour_orn))
        self.pre_pour_orn_axis = np.array(self.pre_pour_orn) / self.pre_pour_orn_angle

        # Rotation matrix
        self.pre_pour_orn_mat = angle2rotm(self.pre_pour_orn_angle, self.pre_pour_orn_axis)

        # Z axis for offset
        self.pre_pour_z_axis = self.pre_pour_orn_mat[:, 2]
        self.gripper_offset = 0.12 # 0.08
        self.gripper_ee_offset = -self.gripper_offset * self.pre_pour_z_axis

    def pick(self):
        # Move to the top of the bottle
        self.robot.move_to_joint(self.pick_bottle_joints[1])

        # Move the robot down
        self.robot.move_to_joint(self.pick_bottle_joints[0])
        
        # Close the gripper
        self.robot.close_gripper()

        # Move the robot up
        self.robot.move_to_joint(self.pick_bottle_joints[1])

        # robot go home
        self.robot.go_home()
    

    def pour(self, pour_pos):
        pre_pour_pos_x = pour_pos[0] + self.gripper_ee_offset[0]
        pre_pour_pos_y = pour_pos[1] + self.gripper_ee_offset[1]
        pre_pour_pos_z = pour_pos[2] + 0.14 # 0.185
        pre_pour_pos = [pre_pour_pos_x, pre_pour_pos_y, pre_pour_pos_z]
        
        self.robot.go_home()

        print(pre_pour_pos)
        print(self.pre_pour_orn)

        self.robot.move_to(pre_pour_pos, self.pre_pour_orn)

        time.sleep(1)

        current_config = self.robot.get_config()
        pour_config = current_config
        pour_config[-1] -= np.pi

        self.robot.move_to_joint(pour_config, acc=12.0, vel=12.0)

        # Shake the bottle
        rotate_angle = np.pi/180 * 8
        shake_config = pour_config
        shake_config_last = shake_config[-1]
        for i in range(10):
            if i % 2 == 0:
                shake_config[-1] = shake_config_last + rotate_angle
                self.robot.move_to_joint(shake_config, acc=12.0, vel=12.0)
            else:
                shake_config[-1] = shake_config_last - rotate_angle
                self.robot.move_to_joint(shake_config, acc=12.0, vel=12.0)
            


        self.robot.disconnect()

    def disconnect(self):
        self.robot.disconnect()


# if __name__ == "__main__":
#     PP = PickAndPour(0.3, 0.3)
#     PP.pick()
#     pour_pos = [-0.09652546464564028, -0.2380368459759542, 0.20159800696372998]
#     PP.pour(pour_pos)



