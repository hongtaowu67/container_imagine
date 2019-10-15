#! /usr/bin/env python

"""
Kuka iiwa with gripper grasping.
@author: Hongtao Wu
Oct 14, 2019
"""

import pybullet as p
import os
import math
import numpy as np
import time
from time import sleep
from kuka_iiwa.kuka import Kuka
from kukawithbaxter import KukaXBaxtor


def eul_to_np_matrix(euler_angle):

    '''
    :param euler_angle: Euler angles in tuple shape
    :return: Rotational matrix in (3, 3) np.array
    '''

    matrix = p.getMatrixFromQuaternion(p.getQuaternionFromEuler(euler_angle))
    return np.reshape(np.array(matrix), (3, 3))


def qua_to_np_matrix(quaternion):

    '''
    :param quaternion: Quaternion in tuple shape
    :return: Rotational matrix in (3, 3) np.array
    '''

    matrix = p.getMatrixFromQuaternion(quaternion)
    return np.reshape(np.array(matrix), (3, 3))


def normal_qua_to_bullet_qua(normal_qua):
    normal_qua_in = np.copy(normal_qua)
    normal_qua_in = np.ndarray.tolist(normal_qua_in)
    bullet_qua = normal_qua_in[1:]
    bullet_qua.append(normal_qua[0])
    return bullet_qua


def apply_action(robot, motor_commands, end_effector_orn_qrt=None):
    '''
    :param motor_commands: a 5x1 list containing (x, y, z, end_effector_rotation_angle, gripper_open_angle)
    :param end_effector_orn_qrt: end_effector_orn in quaternion
    :return: None
    '''
    action = np.array(motor_commands) / 1000
    action[-1] *= 1000
    for i in range(1100):
        if i <= 1000:
            robot.applyAction(action)
        p.stepSimulation()
        sleep(1. / 480.)


def hg_matrix(rotation_matrix, translation_vector):
    hg_matrix = np.append(rotation_matrix, [[0, 0, 0]], axis=0)
    hg_matrix = np.append(hg_matrix, [[translation_vector[0]], [translation_vector[1]], [translation_vector[2]], [1]], axis=1)

    return hg_matrix


def qua_multiplication(qua1_origin, qua2_origin):
    '''
    :param qua1: x1, y1, z1, w1
    :param qua2: x2, y2, z2, w2
    :return: x, y, z, w
    '''
    qua1 = np.copy(np.array(qua1_origin))
    qua2 = np.copy(np.array(qua2_origin))

    b1 = qua1[0]
    c1 = qua1[1]
    d1 = qua1[2]
    a1 = qua1[3]

    b2 = qua2[0]
    c2 = qua2[1]
    d2 = qua2[2]
    a2 = qua2[3]

    w = a1*a2 - b1*b2 - c1*c2 - d1*d2
    x = a1*b2 + a2*b1 + c1*d2 - c2*d1
    y = a1*c2 + a2*c1 + b2*d1 - b1*d2
    z = a1*d2 + a2*d1 + b1*c2 - b2*c1

    return [x, y, z, w]


def main(rootPath=None):
    try:
        p.connect(p.GUI)
        p.setGravity(0, 0, -10)

        kk = KukaXBaxtor(rootPath)
        p.resetDebugVisualizerCamera(1.6, 75, -50, [0, 0, 0])

        # kk.debug()
        kk.loadObject('/object/cube.urdf', [0.5, 0, 0.05])
        
        gripperCurrState = kk.getObservation()
        print('gripper pos: {}'.format(gripperCurrState))

        action = [-0.05, 0, 0, 0, 0.3]
        apply_action(kk, action)

        action = [0, -0.02, -0.30, math.pi/2, 0.3]
        apply_action(kk, action)

        action = [0, 0, 0, 0, 0.01]
        apply_action(kk, action)

        action = [0, 0, 0.18, 0, 0.01]
        apply_action(kk, action)

        action = [-0.3, -0.3, 0, 0, 0.01]
        apply_action(kk, action)

        action = [-0.4, -0.5, 0, 0, 0.01]
        apply_action(kk, action)

        kk.test()

        print("Finish!")
        p.disconnect()
    except:
        p.disconnect()


if __name__ == "__main__":
    rootPath = "/home/hongtao/src/cup_imagine"
    save_mp4_dir = "/home/hongtao/Dropbox/spirit-dictionary/mp4"
    record_process = False

    main(rootPath)
