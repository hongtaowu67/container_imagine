#! /usr/bin/env python
"""
Imaginebot grasps objects. The framework for grasping is TBD.
@author: Hongtao Wu
10/13/2019
"""

import os
import time
import numpy as np
import math
import pybullet as p
import pybullet_data
from ur5_robot.imaginebot_withgripper import ImaginebotWithGripper

def main():
    # start simulation
    try:
        robotUrdfPath = "./ur5_robot/urdf/imaginebot.urdf"
        rob = ImaginebotWithGripper(robotUrdfPath)

        # Load cube
        rob.loadURDF("./object/cube.urdf", [0.5, 0.1, 0.05])

        
        record_process = False 
        # Save mp4 video
        if record_process:
            save_mp4_dir = '/home/hongtao/Desktop'
            today = time.strftime("%Y-%m-%d-%H-%M-%S", time.localtime())
            mp4FileName = 'ur5_test.mp4'
            mp4FilePath = os.path.join(save_mp4_dir, mp4FileName)
            p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, mp4FilePath)

        # rob.debug()
        rob.test(100)

        finger_pos, finger_orn =  rob.readFingerCenterState()
        target_finger_pos = np.array([0.5, 0.1, 0.15])
        target_finger_orn = np.array(p.getQuaternionFromEuler([0, math.pi / 2, 0]))
        rob.go_to(target_finger_pos, target_finger_orn)
        target_finger_pos = [0.5, 0.1, 0.08]
        target_finger_orn = p.getQuaternionFromEuler([0, math.pi / 2, 0])
        rob.go_to(target_finger_pos, target_finger_orn)
        rob.close_gripper()

        target_finger_pos = np.array([0.5, 0.1, 0.15])
        target_finger_orn = np.array(p.getQuaternionFromEuler([0, math.pi / 2, 0])) 
        rob.go_to(target_finger_pos, target_finger_orn)
        target_finger_pos = [0.0, -0.5, 0.15]
        target_orn = p.getQuaternionFromEuler([0, math.pi/2, -math.pi/2])
        rob.go_to(target_finger_pos, target_orn)
        rob.open_gripper()
        
        rob.close_gripper()

        rob.test()

        p.disconnect()
    except:
        p.disconnect()

if __name__ == "__main__":
    main()
