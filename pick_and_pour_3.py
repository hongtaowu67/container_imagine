"""
Pour with different orientations and positions

[tick] TODO #1: pick up an upright bottle 
TODO #2: test pouring about a fixed point
TODO #3: test pour from different orientations
"""

from __future__ import division

import os
import time

import numpy as np
import cv2
import rospy

from robot import Robot
from reconstruction.ros_camera_tsdf_fusion import ROSCameraTSDFFusion
from utils import angle2rotm, make_rigid_transformation, rotm2angle




class PickAndPour:
    def __init__ (self, acc, vel):
        self.acc = acc
        self.vel = vel

        self.robot = Robot(acc=self.acc, vel=self.vel, gripper_on=True, go_home=True)

        self.pick_bottle_links = [
            # Move to the bottle top
            (0.26572323416936444, -0.20275253795367637, 0.7078531195908105, 2.0299297759259516, -2.1059345534142935, 1.0304060872514393),
            # Align the ee with the bottle
            (0.30035709862309257, -0.23779897928815466, 0.48355924356743263, 1.1105965952328265, 1.110547025211203, -0.00017257292410390916),
            # Pick the bottle
            (0.30037875345274245, -0.23780397011114007, 0.18357043252396407, 1.1105965952328265, 1.110547025211203, -0.00017257292410390916),
            # Raise the bottle up
            (0.30037875345274245, -0.23780397011114007, 0.18357043252396407+0.3, 1.1105965952328265, 1.110547025211203, -0.00017257292410390916),
            # Rotate the bottle to horizontal
            (0.30037875345274245, -0.23780397011114007, 0.18357043252396407+0.3, 1.48096098, 1.48096098,  0.0),
            # Rotate the bottle around
            (0.30037875345274245, -0.23780397011114007, 0.18357043252396407+0.3, -2.14574769,  2.14574769,  0.81310401)
        ]

        self.pick_bottle_joint = [
            # Move to the bottle top
            (2.1296579837799072, -1.8189194838153284, 1.6174530982971191, 0.1988968849182129, 1.343645453453064-np.pi/2, -0.7743986288653772),
            # Raise the bottle up
            (2.1296579837799072, -1.8189194838153284, 1.6174530982971191, 0.1988968849182129, 1.343645453453064, -0.7743986288653772),
            # Rotate the bottle around
            (2.1296579837799072, -1.8189194838153284, 1.6174530982971191, 0.1988968849182129, 1.343645453453064-np.pi/2, -0.7743986288653772),
            # Rotate the bottle to horizontal
            (2.1975133419036865, -1.698341194783346, 1.4599494934082031, -0.8148558775531214, -1.6408804098712366, -0.9291647116290491)

        ]
        
        # Pour angles: pre, mid, end
        self.pre_pour_rotate_angle = np.pi/20
        self.mid_pour_rotate_angle = np.pi/4
        self.end_pour_rotate_angle = -np.pi/3
        self.pour_rotate_intervals = 10

        # start pour
        # Unit: angle -pi/20
        self.pre_pour_orn = [3.02589579,  0.0, -0.16839265]
        self.pre_pour_orn_angle = np.linalg.norm(np.array(self.pre_pour_orn))
        self.pre_pour_orn_axis = np.array(self.pre_pour_orn) / self.pre_pour_orn_angle

        # Rotation matrix
        self.pre_pour_orn_mat = angle2rotm(self.pre_pour_orn_angle, self.pre_pour_orn_axis)

        # axis for offset
        self.pre_pour_x_axis = self.pre_pour_orn_mat[:, 0]
        self.pre_pour_y_axis = self.pre_pour_orn_mat[:, 1]
        self.pre_gripper_offset = -0.06
        self.pre_gripper_ee_offset = self.pre_gripper_offset * self.pre_pour_x_axis * np.sin(np.pi/4) - self.pre_gripper_offset * self.pre_pour_y_axis * np.cos(np.pi/4)
        self.pre_gripper_ee_offset_z = 0.20

        # mid pour -pi/4
        self.mid_pour_orn = [2.48899947, 0.0, -0.72901107]
        self.mid_pour_orn_angle = np.linalg.norm(np.array(self.mid_pour_orn))
        self.mid_pour_orn_axis = np.array(self.mid_pour_orn) / self.mid_pour_orn_angle
        # Rotation matrix
        self.mid_pour_orn_mat = angle2rotm(self.mid_pour_orn_angle, self.mid_pour_orn_axis)
        self.mid_pour_z_axis = self.mid_pour_orn_mat[:, 2]
        self.mid_gripper_offset = -0.07
        self.mid_gripper_ee_offset = self.mid_pour_z_axis * self.mid_gripper_offset
        self.mid_gripper_ee_offset_z = 0.20

        # end pour -2*pi/5
        self.end_pour_orn = [2.03187381,  0.0, -1.04386125]
        self.end_pour_orn_angle = np.linalg.norm(np.array(self.end_pour_orn))
        self.end_pour_orn_axis = np.array(self.end_pour_orn) / self.end_pour_orn_angle

        self.end_pour_orn_mat = angle2rotm(self.end_pour_orn_angle, self.end_pour_orn_axis)
        self.end_pour_z_axis = self.end_pour_orn_mat[:, 2]
        self.end_gripper_offset = -0.14
        self.end_gripper_ee_offset = self.end_gripper_offset * self.end_pour_z_axis
        self.end_gripper_ee_offset_z = 0.15


    # Pick horizontal bottle
    def pick_horizontal(self):
        # Move to the top of the bottle
        self.robot.move_to_joint(self.pick_bottle_joints[1])
        # Move the robot down
        self.robot.move_to_joint(self.pick_bottle_joints[0])
        # Close the gripper
        self.robot.close_gripper()
        # Move the robot up
        self.robot.move_to_joint(self.pick_bottle_joints[1])
        # Turn the bottle
        self.robot.move_to_joint(self.pick_bottle_joints[2])
        self.robot.move_to_joint(self.pick_bottle_joints[3])


    # Pick vertical bottle
    def pick_vertical(self):
        # Move to the bottle top
        # self.robot.move_to_joint(self.pick_bottle_joint[0])
        # self.robot.move_to(self.pick_bottle_links[0][:3], self.pick_bottle_links[0][3:], self.acc, self.vel)
        # Align the ee with the bottle
        self.robot.move_to_joint(self.pick_bottle_joint[1])
        # self.robot.move_to(self.pick_bottle_links[1][:3], self.pick_bottle_links[1][3:], self.acc, self.vel)
        # Pick the bottle
        self.robot.move_to(self.pick_bottle_links[2][:3], self.pick_bottle_links[2][3:], self.acc, self.vel)
        # Close the gripper
        self.robot.close_gripper()
        # Raise the bottle up
        self.robot.move_to_joint(self.pick_bottle_joint[1])
        # self.robot.move_to(self.pick_bottle_links[3][:3], self.pick_bottle_links[3][3:], self.acc, self.vel)
        # Turn the bottle around
        self.robot.move_to_joint(self.pick_bottle_joint[2])
        self.robot.move_to_joint(self.pick_bottle_joint[3])
        # self.robot.move_to(self.pick_bottle_links[4][:3], self.pick_bottle_links[4][3:], self.acc, self.vel)
        # self.robot.move_to(self.pick_bottle_links[5][:3], self.pick_bottle_links[5][3:], self.acc, self.vel)

    
    # Pouring from a single planar angle
    def pour_single_orn(self, pour_pos):

        pre_pour_pos_x = pour_pos[0] + self.pre_gripper_ee_offset[0]
        pre_pour_pos_y = pour_pos[1] + self.pre_gripper_ee_offset[1]
        pre_pour_pos_z = pour_pos[2] + self.pre_gripper_ee_offset_z
        pre_pour_pos = [pre_pour_pos_x, pre_pour_pos_y, pre_pour_pos_z]
        print "pre_pour_pos: ", pre_pour_pos
        self.robot.move_to(pre_pour_pos, self.pre_pour_orn, acc=0.5, vel=0.5)

        # time.sleep(1)

        mid_pour_pos_x = pour_pos[0] + self.mid_gripper_ee_offset[0]
        mid_pour_pos_y = pour_pos[1] + self.mid_gripper_ee_offset[1]
        mid_pour_pos_z = pour_pos[2] + self.mid_gripper_ee_offset_z
        mid_pour_pos = [mid_pour_pos_x, mid_pour_pos_y, mid_pour_pos_z]
        print "mid_pour_pos: ", mid_pour_pos
        self.robot.move_to(mid_pour_pos, self.mid_pour_orn, acc=0.1, vel=0.1)

        end_pour_pos_x = pour_pos[0] + self.end_gripper_ee_offset[0]
        end_pour_pos_y = pour_pos[1] + self.end_gripper_ee_offset[1]
        end_pour_pos_z = pour_pos[2] + self.end_gripper_ee_offset_z
        end_pour_pos = [end_pour_pos_x, end_pour_pos_y, end_pour_pos_z]
        print "end_pour_pos: ", end_pour_pos
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
    

    # Pouring from the best pouring planar_angle
    # The planar angle is w.r.t the y-axis of the robot
    def pour_multi_orn(self, imagined_pour_pos, bottle_angle=np.pi/4):
        """
        Pour with multiple bottle angle. Does not work for np.pi/2
        """
        for i in range(self.pour_rotate_intervals + 1):
            pour_angle = i / self.pour_rotate_intervals * self.end_pour_rotate_angle
            pour_pos = self.get_pour_pos(pour_angle, imagined_pour_pos, bottle_angle)
            pour_orn = self.get_pour_orn(pour_angle, bottle_angle)
            self.robot.move_to(pour_pos.tolist(), pour_orn.tolist(), acc=0.1, vel=0.1)
            print "pour_pos: ", pour_pos
            print "pour_orn: ", pour_orn

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
        
        print "Finish pouring..."
        self.robot.disconnect()
    

    def get_pour_orn(self, pour_angle, bottle_angle=np.pi/4):
        """
        Calculate the axis-angle rotation of the ee for a given pour_angle and bottle angle.
        Bottle angle is in se(2).
        """        
        # Base rotation in the world frame
        base_rotm = np.array([[np.cos(bottle_angle + 3*np.pi/4), np.cos(bottle_angle + np.pi/4), 0.0],
                              [np.sin(bottle_angle + 3*np.pi/4), np.sin(bottle_angle + np.pi/4), 0.0],
                              [0, 0, -1]])

        rotate_axis = np.array([np.cos(bottle_angle - np.pi/2), np.sin(bottle_angle - np.pi/2), 0.0])
    
        # Rotation added on the base rotation in the world frame
        added_rotm = angle2rotm(pour_angle, rotate_axis)
        added_rotm = added_rotm[:3, :3]

        # Combined rotation in the world frame
        # The added rotation is in the world frame thus R_added * R_base
        pour_axis_angle = rotm2angle(np.dot(added_rotm, base_rotm))
        pour_axis_angle = np.array(pour_axis_angle)

        pour_angle = pour_axis_angle[0]
        pour_axis = pour_axis_angle[1:]

        return pour_angle * pour_axis

    
    def get_pour_pos(self, pour_angle, imagined_pour_pos, bottle_angle=np.pi/4):
        """
        Rotation about a fixed point. Given a pour angle, calculate the position 
        of the ee for rotating about a fixed point. The fixed point is the imagined
        pouring position.

        Args:
            - bottle_angle: bottle opening pointing in se(2)
            - pour_angle: rotating angle, e.g. -np.pi/4
        """
        # bottle lower tips in the ee frame
        ee_offset_x = 0.07 * np.cos(3*np.pi/4)
        ee_offset_y = 0.07 * np.sin(3*np.pi/4)
        ee_offset_z = 0.195
        ee_offset = np.array([ee_offset_x, ee_offset_y, ee_offset_z])

        # ee frame in the world frame when bottle is placed horizontal
        bottle_base_rotm = np.array([[np.cos(bottle_angle + 3*np.pi/4), np.cos(bottle_angle + np.pi/4), 0.0],
                                     [np.sin(bottle_angle + 3*np.pi/4), np.sin(bottle_angle + np.pi/4), 0.0],
                                     [0, 0, -1]])
        
        # Applied rotation to the ee horizontal frame
        bottle_rotate_axis = np.array([np.cos(bottle_angle - np.pi/2), np.sin(bottle_angle - np.pi/2), 0])
        bottle_rotate_angle = pour_angle
        bottle_rotate_rotm = angle2rotm(bottle_rotate_angle, bottle_rotate_axis)[:3, :3]

        # ee frame rotation w.r.t. the world frame
        bottle_pour_rotm = np.dot(bottle_rotate_rotm, bottle_base_rotm)
        
        # ee position in the world frame
        # imagined_pour_pos = R * ee_offset + t
        bottle_pour_pos = imagined_pour_pos - np.dot(bottle_pour_rotm, ee_offset)

        return bottle_pour_pos


    def disconnect(self):
        self.robot.disconnect()


if __name__ == "__main__":
    PP = PickAndPour(0.5, 0.5)
    PP.pick_vertical()
    pour_pos = [-0.0955295, -0.29171583, 0.202606]

    # PP.pour_single_orn(pour_pos)
    PP.pour_multi_orn(pour_pos, bottle_angle=np.pi)

    # pre_pour_angle = -np.pi/20
    # mid_pour_angle = -np.pi/4
    # end_pour_angle = -2*np.pi/5

    # pre_pour_pos = PP.get_pour_pos(pre_pour_angle, np.array(pour_pos))
    # pre_pour_orn = PP.get_pour_orn(pre_pour_angle)
    # print "New pre pour pos: ", pre_pour_pos
    # print "New pre pour orn: ", pre_pour_orn
    # PP.robot.move_to(pre_pour_pos.tolist(), pre_pour_orn.tolist(), acc=0.1, vel=0.1)

    # mid_pour_pos = PP.get_pour_pos(mid_pour_angle, np.array(pour_pos))
    # mid_pour_orn = PP.get_pour_orn(mid_pour_angle)
    # print "New mid pour pos: ", mid_pour_pos
    # print "New mid pour orn: ", mid_pour_orn
    # PP.robot.move_to(mid_pour_pos.tolist(), mid_pour_orn.tolist(), acc=0.1, vel=0.1)
    
    # end_pour_pos = PP.get_pour_pos(end_pour_angle, np.array(pour_pos))
    # end_pour_orn = PP.get_pour_orn(end_pour_angle)
    # print "New end pour pos: ", end_pour_pos
    # print "New end pour orn: ", end_pour_orn
    # PP.robot.move_to(end_pour_pos.tolist(), end_pour_orn.tolist(), acc=0.1, vel=0.1)

    # PP.disconnect()

# >>> r1 = utils.angle2rotm(np.pi, (1.0, 0.0, 0.0)) #Angle when the ee is pointing downward
# >>> r2 = utils.angle2rotm(-np.pi*2/5, np.array([np.sin(np.pi/4), -np.cos(np.pi/4), 0])) #Angle offset when the bottle is rotating from
# >>> axis = utils.rotm2angle(np.dot(r2, r1)[:3, :3])
# >>> axis = np.array(axis)
# >>> axis = axis[1:] * axis[0]



    # Deprecated 
    # def get_pour_orn(self, pour_angle, bottle_angle=np.pi/4):
    #     """
    #     Calculate the axis-angle rotation of the ee for a given pour_angle and bottle angle.
    #     Bottle angle is in se(2).
    #     """        
    #     # Base rotation in the world frame
    #     base_rotm = np.array([[np.cos(bottle_angle - np.pi/4),  np.sin(bottle_angle - np.pi/4), 0.0],
    #                                  [np.sin(bottle_angle - np.pi/4), -np.cos(bottle_angle - np.pi/4), 0.0],
    #                                  [0, 0, -1]])

    #     rotate_axis = np.array([np.cos(bottle_angle - np.pi/2), np.sin(bottle_angle - np.pi/2), 0.0])
    
    #     # Rotation added on the base rotation in the world frame
    #     added_rotm = angle2rotm(pour_angle, rotate_axis)
    #     added_rotm = added_rotm[:3, :3]

    #     # Combined rotation in the world frame
    #     # The added rotation is in the world frame thus R_added * R_base
    #     pour_axis_angle = rotm2angle(np.dot(added_rotm, base_rotm))
    #     pour_axis_angle = np.array(pour_axis_angle)

    #     pour_angle = pour_axis_angle[0]
    #     pour_axis = pour_axis_angle[1:]

    #     return pour_angle * pour_axis

    
    # def get_pour_pos(self, pour_angle, imagined_pour_pos, bottle_angle=np.pi/4):
    #     """
    #     Rotation about a fixed point. Given a pour angle, calculate the position 
    #     of the ee for rotating about a fixed point. The fixed point is the imagined
    #     pouring position.

    #     Args:
    #         - bottle_angle: bottle opening pointing in se(2)
    #         - pour_angle: rotating angle, e.g. -np.pi/4
    #     """
    #     # bottle lower tips in the ee frame
    #     ee_offset_x = 0.07 * np.cos(-np.pi/4)
    #     ee_offset_y = 0.07 * np.sin(-np.pi/4)
    #     ee_offset_z = 0.195
    #     ee_offset = np.array([ee_offset_x, ee_offset_y, ee_offset_z])

    #     # ee frame in the world frame when bottle is placed horizontal
    #     bottle_base_rotm = np.array([[np.cos(bottle_angle - np.pi/4),  np.sin(bottle_angle - np.pi/4), 0.0],
    #                                  [np.sin(bottle_angle - np.pi/4), -np.cos(bottle_angle - np.pi/4), 0.0],
    #                                  [0, 0, -1]])
        
    #     # Applied rotation to the ee horizontal frame
    #     bottle_rotate_axis = np.array([np.cos(bottle_angle - np.pi/2), np.sin(bottle_angle - np.pi/2), 0])
    #     bottle_rotate_angle = pour_angle
    #     bottle_rotate_rotm = angle2rotm(bottle_rotate_angle, bottle_rotate_axis)[:3, :3]

    #     # ee frame rotation w.r.t. the world frame
    #     bottle_pour_rotm = np.dot(bottle_rotate_rotm, bottle_base_rotm)
        
    #     # ee position in the world frame
    #     # imagined_pour_pos = R * ee_offset + t
    #     bottle_pour_pos = imagined_pour_pos - np.dot(bottle_pour_rotm, ee_offset)

    #     return bottle_pour_pos

    # Pick up the bottle the other way
    # self.pick_bottle_links = [
    #     # Move to the bottle top
    #     (0.26572323416936444, -0.20275253795367637, 0.7078531195908105, 2.0299297759259516, -2.1059345534142935, 1.0304060872514393),
    #     # Align the ee with the bottle
    #     (0.30037875345274245, -0.23780397011114007, 0.18357043252396407+0.3, 1.5706662714158346, -1.5705926855260797, 2.2215507836252812),
    #     # Pick the bottle
    #     (0.30037875345274245, -0.23780397011114007, 0.18357043252396407, 1.5706662714158346, -1.5705926855260797, 2.2215507836252812),
    #     # Raise the bottle up
    #     (0.30037875345274245, -0.23780397011114007, 0.18357043252396407+0.3, 1.5706662714158346, -1.5705926855260797, 2.2215507836252812),
    #     # Turn the bottle around
    #     (0.2808985287976357, -0.23254415673779724, 0.36694994918696516+0.1, -2.0890063575780045, -2.0673261506314793, -0.05046204744094082)
    # ]





