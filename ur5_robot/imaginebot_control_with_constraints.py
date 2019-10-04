#!/usr/bin/env python
'''

Imaginebot basic setup and control(ur5 robot + robotiq 2f-85 gripper).
@author: Hongtao Wu
10/03/2019

'''

import os
import time
import pdb
import pybullet as p
import pybullet_data
from collections import namedtuple
from attrdict import AttrDict
import math


class Robot(object):
    def __init__(self, robot_urdf):
        super(Robot, self).__init__()

        self.serverMode = p.GUI # GUI/DIRECT

        # connect to engine servers
        physicsClient = p.connect(self.serverMode)
        # add search path for loadURDF
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        # define world
        p.setGravity(0,0,-10)
        # The plane could be interfering with the robot
        self.planeID = p.loadURDF("plane.urdf")

        # Joint ID
        self.jointID = {'world_joint': 0,
                        'shoulder_pan_joint': 1,
                        'shoulder_lift_joint': 2,
                        'elbow_joint': 3,
                        'wrist_1_joint': 4,
                        'wrist_2_joint': 5,
                        'wrist_3_joint': 6,
                        'ee_fixed_joint': 7}

        # Joint for control
        self.controlJoints = ["shoulder_pan_joint",
                              "shoulder_lift_joint",
                              "elbow_joint", 
                              "wrist_1_joint",
                              "wrist_2_joint", 
                              "wrist_3_joint"]

        # Initial joint value
        self.initialJointValue = {'world_joint': 0,
                                  'shoulder_pan_joint': 0,
                                  'shoulder_lift_joint': -math.pi/2,
                                  'elbow_joint': 0,
                                  'wrist_1_joint': 0,
                                  'wrist_2_joint': 0,
                                  'wrist_3_joint': 0,
                                  'ee_fixed_joint': 0}
        
        # In the simulation, we consider the torque to be large
        self.maxJointForce = 150

        # Robot start position and orientation
        robotStartPos = [0,0,0]
        robotStartOrn = p.getQuaternionFromEuler([0,0,0])

        # Load the robot urdf file
        self.robotID = p.loadURDF(robotUrdfPath, robotStartPos, robotStartOrn, 
                            flags=p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT) # This will discord self-collision between a child link and any of its ancestors.


        # Get Robot Joint Number
        self.numJoints = p.getNumJoints(self.robotID) # The robot has 18 joints (including the gripper)
        print('Number of joint:{}'.format(self.numJoints))

        # Set the initial joint angle of each joint
        for joint_name in self.initialJointValue.keys():
            p.resetJointState(self.robotID, self.jointID[joint_name], self.initialJointValue[joint_name])
            p.setJointMotorControl2(self.robotID, self.jointID[joint_name], p.POSITION_CONTROL, targetPosition=self.initialJointValue[joint_name], force=self.maxJointForce)
        
        ##############################
        # Get Robot Joint Information
        # jointInfoList = [p.getJointInfo(self.robotID, jointID) for jointID in range(self.numJoints)]
        # for jointInfo in jointInfoList:
        #     print(jointInfo)
        ##############################

        jointTypeList = ["REVOLUTE", "PRISMATIC", "SPHERICAL", "PLANAR", "FIXED"]

        jointInfo = namedtuple("jointInfo", 
                            ["id","name","type","lowerLimit","upperLimit","maxForce","maxVelocity","controllable"])
        self.joints = AttrDict()
        for i in range(self.numJoints):
            info = p.getJointInfo(self.robotID, i)
            jointID = info[0]
            jointName = info[1].decode("utf-8")
            jointType = jointTypeList[info[2]]
            jointLowerLimit = info[8]
            jointUpperLimit = info[9]
            jointMaxForce = info[10]
            print('{} Max Force: {}'.format(jointName, jointMaxForce))
            jointMaxVelocity = info[11]
            controllable = True if jointName in self.controlJoints else False
            info = jointInfo(jointID,jointName,jointType,jointLowerLimit,
                            jointUpperLimit,jointMaxForce,jointMaxVelocity,controllable)
            
            # if info.type=="REVOLUTE": # set revolute joint to static
            #     p.setJointMotorControl2(robotID, info.id, p.VELOCITY_CONTROL, targetVelocity=0, force=0)
            self.joints[info.name] = info

    def test(self):
        while True:
            p.stepSimulation()

    def debug(self):
        userParams = dict()
        for name in self.controlJoints:
            joint = self.joints[name]
            userParam = p.addUserDebugParameter(name, joint.lowerLimit, joint.upperLimit, 0)
            userParams[name] = userParam
        while(flag):
            for name in self.controlJoints:
                joint = self.joints[name]
                pose = p.readUserDebugParameter(userParams[name])
                p.setJointMotorControl2(self.robotID, joint.id, p.POSITION_CONTROL,
                                        targetPosition=pose, force=joint.maxForce, 
                                        maxVelocity=joint.maxVelocity)
            p.stepSimulation()

    # ###############################################
    # ## set up mimic joints in robotiq_c2 gripper ##
    # ###############################################
    # mimicParentName = "robotiq_85_left_knuckle_joint"
    # mimicChildName = ["robotiq_85_right_knuckle_joint",
    #                   "robotiq_85_right_finger_joint",
    #                   "robotiq_85_left_inner_knuckle_joint",
    #                   "robotiq_85_left_finger_tip_joint",
    #                   "robotiq_85_right_inner_knuckle_joint",
    #                   "robotiq_85_right_finger_tip_joint"]
    # mimicMul = [-1,-1,-1,-1,-1,-1]
    # mimicChildList = []
    # parent = self.joints[mimicParentName]
    # constraints = dict()
    # for i, name in enumerate(mimicChildName):
    #     child = joints[name]
    #     c = p.createConstraint(self.robotID, parent.id,
    #                            self.robotID, child.id,
    #                            jointType=p.JOINT_GEAR,
    #                            jointAxis=[0,0,1],
    #                            parentFramePosition=[0,0,0],
    #                            childFramePosition=[0,0,0])
    #     p.changeConstraint(c, gearRatio=mimicMul[i], maxForce=child.maxForce)
    #     constraints[name] = c


if __name__ == "__main__":

    # start simulation
    try:
        flag = True
        robotUrdfPath = "./urdf/imaginebot_nogripper.urdf"
        rob = Robot(robotUrdfPath)
        rob.test()
        p.disconnect()
    except:
        p.disconnect()

