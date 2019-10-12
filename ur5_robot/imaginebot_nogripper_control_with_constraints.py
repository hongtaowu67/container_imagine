#!/usr/bin/env python
'''

Imaginebot basic setup and control(ur5 robot).
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


class RobotNoGripper(object):
    def __init__(self, robot_urdf):
        super(RobotNoGripper, self).__init__()

        self.serverMode = p.GUI # GUI/DIRECT

        # connect to engine servers
        physicsClient = p.connect(self.serverMode)
        # add search path for loadURDF
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        # define world
        p.setGravity(0,0,-10)
        # The plane could be interfering with the robot
        # self.planeID = p.loadURDF("plane.urdf")

        # Joint ID
        self.jointID = {'world_joint': 0,
                        'shoulder_pan_joint': 1,
                        'shoulder_lift_joint': 2,
                        'elbow_joint': 3,
                        'wrist_1_joint': 4,
                        'wrist_2_joint': 5,
                        'wrist_3_joint': 6,
                        'ee_fixed_joint': 7}
        
        # End Effector ID
        self.eeID = self.jointID['ee_fixed_joint']

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
                                  'elbow_joint': math.pi/2,
                                  'wrist_1_joint': -math.pi/2,
                                  'wrist_2_joint': -math.pi/2,
                                  'wrist_3_joint': 0,
                                  'ee_fixed_joint': 0}

         # Joints range
        self.jointsRange = [2*math.pi, 2*math.pi, 2*math.pi, 2*math.pi, 2*math.pi, 2*math.pi]

        # Joints restpose
        self.jointsRestPose = [math.pi/4, -math.pi/2, math.pi/2, -math.pi/2, -math.pi/2, 0]       

        # In the simulation, we consider the torque to be large
        self.maxJointForce = 150

        # jointLimit
        self.jointsLowerLimit = []
        self.jointsUpperLimit = []

        # Robot start position and orientation
        robotStartPos = [0,0,0]
        robotStartOrn = p.getQuaternionFromEuler([0,0,0])


        # Load the robot urdf file
        self.robotID = p.loadURDF(robotUrdfPath, robotStartPos, robotStartOrn, 
                            flags=p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT) # This will discord self-collision between a child link and any of its ancestors.


        # Get robot joint number
        self.numJoints = p.getNumJoints(self.robotID) # The robot has 18 joints (including the gripper)
        print('Number of joint:{}'.format(self.numJoints))

        # Set the initial joint angle of each joint
        for joint_name in self.initialJointValue.keys():
            p.resetJointState(self.robotID, self.jointID[joint_name], self.initialJointValue[joint_name])
            p.setJointMotorControl2(self.robotID, self.jointID[joint_name], p.POSITION_CONTROL, targetPosition=self.initialJointValue[joint_name], force=self.maxJointForce)
        
        ##############################
        # Get Robot Joint Information
        jointInfoList = [p.getJointInfo(self.robotID, jointID) for jointID in range(self.numJoints)]
        for jointInfo in jointInfoList:
            print(jointInfo)
        ##############################

        # Robot joint information
        # It can be called by self.joints[joint_name].id/type/lowerLimit...
        jointTypeList = ["REVOLUTE", "PRISMATIC", "SPHERICAL", "PLANAR", "FIXED"]

        jointInfo = namedtuple("jointInfo", ["id","name","type","lowerLimit","upperLimit","maxForce","maxVelocity","controllable"])
        self.joints = AttrDict()
        for i in range(self.numJoints):
            info = p.getJointInfo(self.robotID, i)
            jointID = info[0]
            jointName = info[1].decode("utf-8")
            jointType = jointTypeList[info[2]]
            jointLowerLimit = info[8]
            jointUpperLimit = info[9]

            if jointName in self.controlJoints:
                self.jointsLowerLimit.append(jointLowerLimit)
                self.jointsUpperLimit.append(jointUpperLimit)

            jointMaxForce = info[10]
            print('{} Max Force: {}'.format(jointName, jointMaxForce))
            jointMaxVelocity = info[11]
            controllable = True if jointName in self.controlJoints else False
            info = jointInfo(jointID,jointName,jointType,jointLowerLimit,
                            jointUpperLimit,jointMaxForce,jointMaxVelocity,controllable)
            
            self.joints[info.name] = info


    def readJointState(self, jointID):
        '''
        Return the state of the joint: jointPosition, jointVelocity, jointReactionForces, appliedJointMotroTorque
        '''
        return p.getJointState(self.robotID, jointID)


    def readEndEffectorState(self):
        '''
        Return the position and orientation (quaternion) of the endeffector
        '''
        ee_state = p.getLinkState(self.robotID, self.eeID)
        
        return ee_state[0], ee_state[1]
    

    def goto(self, pos, orn=None):
        '''
        pos: list of 3 floats
        orn: list of 4 floats, in quaternion
        Make the end-effector reach a given target position in Cartesian world space.
        '''


        if orn:
            jointTargetPose_list = p.calculateInverseKinematics(self.robotID,
                                                                self.eeID, 
                                                                targetPosition=pos,
                                                                targetOrientation=orn,
                                                                lowerLimits=self.jointsLowerLimit,
                                                                upperLimits=self.jointsUpperLimit,
                                                                jointRanges=self.jointsRange,
                                                                restPoses=self.jointsRestPose)
                                                                     
            print('joint target pose list: {}'.format(jointTargetPose_list))
        else:
            jointTargetPose_list = p.calculateInverseKinematics(self.robotID, 
                                                                self.eeID, 
                                                                targetPosition=pos,
                                                                lowerLimits=self.jointsLowerLimit,
                                                                upperLimits=self.jointsUpperLimit,
                                                                jointRanges=self.jointsRange,
                                                                restPoses=self.jointsRestPose)

            print('joint target pose list: {}'.format(jointTargetPose_list))
        
        for i in range(500):
            for jointIdx, jointName in enumerate(self.controlJoints):
                jointTargetState = jointTargetPose_list[jointIdx]
                p.setJointMotorControl2(self.robotID, self.joints[jointName].id, p.POSITION_CONTROL,
                                        targetPosition=jointTargetState, force=self.joints[jointName].maxForce,
                                        maxVelocity=self.joints[jointName].maxVelocity)
            p.stepSimulation()
            time.sleep(1./240.)


    def test(self, sim_timesteps=None):
        '''
        sim_timesteps: if None, then simulate forever; if not, simulate for the sim_timesteps
        '''
        if not sim_timesteps:
            while True:
                p.stepSimulation()
        else:
            for i in range(sim_timesteps):
                p.stepSimulation()
                time.sleep(1./240.)


    def debug(self):
        userParams = dict()
        for name in self.controlJoints:
            joint = self.joints[name]
            userParam = p.addUserDebugParameter(name, joint.lowerLimit, joint.upperLimit, 0)
            userParams[name] = userParam
        while True:
            for name in self.controlJoints:
                joint = self.joints[name]
                pose = p.readUserDebugParameter(userParams[name])
                p.setJointMotorControl2(self.robotID, joint.id, p.POSITION_CONTROL,
                                        targetPosition=pose, force=joint.maxForce, 
                                        maxVelocity=joint.maxVelocity)

            p.stepSimulation()


if __name__ == "__main__":

    # start simulation
    try:
        robotUrdfPath = "./urdf/imaginebot_nogripper.urdf"
        rob = RobotNoGripper(robotUrdfPath)
        
#        rob.debug()

        rob.test(100)

    #     pos, orn = rob.readEndEffectorState()

    #     print('#######################')
    #     print('pos: {} \n'.format(pos))
    #     print('orn in Euler Angle: {} \n'.format(p.getEulerFromQuaternion(orn)))
    #     print('#######################')

    #     time.sleep(2)

        new_pos = [0.6, 0.1, 0.0]
        new_orn = p.getQuaternionFromEuler([0, math.pi/6, 0])

        rob.goto(new_pos, new_orn)

        print(rob.readEndEffectorState())

    #     p.disconnect()
    except:
        p.disconnect()


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


