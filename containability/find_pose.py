import numpy as np
import pybullet as p
import pybullet_data
import time
import os
import math

sphere_urdf = "/home/hongtao/Dropbox/spirit-dictionary/dataset/general_object/sphere_mini.urdf"

def find_pose(*args):
    # Set the world
    physicsClient = p.connect(p.GUI)
    p.setGravity(0, 0, -10)

    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    obj_zero_pos = [0, 0, 0]
    obj_zero_orn = p.getQuaternionFromEuler([0, 0, 0])

    for obj_urdf in args:
        obj_id = p.loadURDF(obj_urdf, basePosition=obj_zero_pos, baseOrientation=obj_zero_orn, useFixedBase=True)

    # Reset debug camera postion
    p.resetDebugVisualizerCamera(0.5, 75, -50, [0, 0, 0])

if __name__ == "__main__":
        # Object information
    model_root_dir = "/home/hongtao/src/cup_imagine/model"
    object_subdir = '1127_cupsmalltapeglass'
    object0_name = object_subdir + '_mesh_debug_0'
    object1_name = object_subdir + '_mesh_debug_1'
    obj0_urdf = os.path.join(model_root_dir, object_subdir, object0_name + '.urdf')
    obj1_urdf = os.path.join(model_root_dir, object_subdir, object1_name + '.urdf')

    find_pose(obj0_urdf, obj1_urdf)
    sphere_urdf = "/home/hongtao/Dropbox/spirit-dictionary/dataset/general_object/sphere_mini.urdf"
    p.loadURDF(sphere_urdf, basePosition=[0.22067185, 0.130192,  0.11523071])
    # p.loadURDF(sphere_urdf, basePosition=[0.22587525, 0.129635,   0.11165962])

    import ipdb; ipdb.set_trace()