#!/usr/bin/env python
"""
Imaginebot basic setup and control(ur5 robot + robotiq 2f-85 gripper).
@author: Hongtao Wu
10/03/2019
"""

import os
import time
import pdb
import pybullet as p
import pybullet_data
from collections import namedtuple
from attrdict import AttrDict
import math
import tools
import numpy as np


class RobotWithGripper(object):
    def __init__(self, robot_urdf):
        super(RobotWithGripper, self).__init__()

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
        self.jointID = {
                        'world_joint': 0,
                        'shoulder_pan_joint': 1,
                        'shoulder_lift_joint': 2,
                        'elbow_joint': 3,
                        'wrist_1_joint': 4,
                        'wrist_2_joint': 5,
                        'wrist_3_joint': 6,
                        'ee_fixed_joint': 7,
                        'arm_gripper_joint': 8,
                        'robotiq_85_base_joint': 9,
                        'robotiq_85_left_knuckle_joint': 10,
                        'robotiq_85_left_finger_joint': 11,
                        'robotiq_85_right_knuckle_joint': 12,
                        'robotiq_85_right_finger_joint': 13,
                        'robotiq_85_left_inner_knuckle_joint': 14,
                        'robotiq_85_left_finger_tip_joint': 15,
                        'robotiq_85_right_inner_knuckle_joint': 16,
                        'robotiq_85_right_finger_tip_joint': 17
                        }
        
        # End Effector ID
        self.eeID = self.jointID['ee_fixed_joint']
        
        # Finger-End Effector offset (in the gripper frame [x, y, z])
        self.ee_finger_offset = [0.10559, 0.0, -0.00410]

        # Joint for control
        self.robotControlJoints = ['shoulder_pan_joint',
                              'shoulder_lift_joint',
                              'elbow_joint', 
                              'wrist_1_joint',
                              'wrist_2_joint', 
                              'wrist_3_joint']

        # Gripper joint for control
        self.gripperControlJoints = ['robotiq_85_left_inner_knuckle_joint',
                                    'robotiq_85_right_inner_knuckle_joint']

        # Finger-EE error tolerance
        self.posErrorTolerance = 0.1  # 10cm
        self.ornErrorTolerance = 0.15 # Approximately 10 deg
        self.jointErrorTolerance = 0.001 # Each joint has 1 deg tolerance on average
        self.maxControlTimeStep = 2400 # each control has at most 10 seconds to execute
        self.gripperControlTimeStep = 200 # about 1s for gripper to open and close

        # Initial joint value
        self.initialJointValue = {
                                  'world_joint': 0,
                                  'shoulder_pan_joint': math.pi/4,
                                  'shoulder_lift_joint': -math.pi/2,
                                  'elbow_joint': math.pi/2,
                                  'wrist_1_joint': -math.pi/2,
                                  'wrist_2_joint': -math.pi/2,
                                  'wrist_3_joint': 0,
                                  'ee_fixed_joint': 0,
                                  'robotiq_85_left_inner_knuckle_joint': 0,
                                  'robotiq_85_right_inner_knuckle_joint': 0
                                  }
        

        # Robot start position and orientation
        robotStartPos = [0,0,0]
        robotStartOrn = p.getQuaternionFromEuler([0,0,0])

        # Load the robot urdf file
        self.robotID = p.loadURDF(robotUrdfPath, robotStartPos, robotStartOrn, 
                            flags=p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT) # This will discord self-collision between a child link and any of its ancestors.


        # Get robot joint number
        self.numJoints = p.getNumJoints(self.robotID) # The robot has 18 joints (including the gripper)

        # Set the initial joint angle of each joint
        for joint_name in self.initialJointValue.keys():
            p.resetJointState(self.robotID, self.jointID[joint_name], self.initialJointValue[joint_name])
            p.setJointMotorControl2(self.robotID, self.jointID[joint_name], p.POSITION_CONTROL, targetPosition=self.initialJointValue[joint_name])
        
        ##############################
        # Get Robot Joint Information
#        jointInfoList = [p.getJointInfo(self.robotID, jointID) for jointID in range(self.numJoints)]
#        for jointInfo in jointInfoList:
#            print(jointInfo)
        ##############################

        # Robot joint information
        # It can be called by self.joints[joint_name].id/type/lowerLimit...
        jointTypeList = ["REVOLUTE", "PRISMATIC", "SPHERICAL", "PLANAR", "FIXED"]

        jointInfo = namedtuple("jointInfo", ["id","name","type","lowerLimit","upperLimit",
                               "maxForce","maxVelocity","controllable"])
        self.joints = AttrDict()

        # Joints lower/upper limit
        self.jointsLowerLimit = []
        self.jointsUpperLimit = []

        # Joints range
        self.jointsRange = [2*math.pi, 2*math.pi, 2*math.pi, 2*math.pi, 2*math.pi, 2*math.pi, 0.4, 0.4]

        # Joints restpose
        self.jointsRestPose = [math.pi/4, -math.pi/2, math.pi/2, -math.pi/2, -math.pi/2, 0, 0, 0]

        for i in range(self.numJoints):
            info = p.getJointInfo(self.robotID, i)
            jointID = info[0]
            jointName = info[1].decode("utf-8")
            jointType = jointTypeList[info[2]]
            jointLowerLimit = info[8]
            jointUpperLimit = info[9]

            if jointName in (self.robotControlJoints + self.gripperControlJoints):
                self.jointsLowerLimit.append(jointLowerLimit)
                self.jointsUpperLimit.append(jointUpperLimit)

            jointMaxForce = info[10]
            jointMaxVelocity = info[11]
            controllable = True if jointName in (self.robotControlJoints + self.gripperControlJoints) else False
            info = jointInfo(jointID,jointName,jointType,jointLowerLimit,
                            jointUpperLimit,jointMaxForce,jointMaxVelocity,controllable)
            
            self.joints[info.name] = info


    def readJointState(self, jointID):
        """
        Return the state of the joint: jointPosition, jointVelocity, jointReactionForces, appliedJointMotroTorque.
        """
        return p.getJointState(self.robotID, jointID)


    def readEndEffectorState(self):
        """
        Return the position and orientation (quaternion) of the endeffector.
        Both position and orientation are numpy arrays.
        """
        ee_state = p.getLinkState(self.robotID, self.eeID)
        
        return np.array(ee_state[0]), np.array(ee_state[1])


    def readFingerCenterState(self):
        """
        Return the position and orientation (quaternion) of the center of the two fingers.
        Both position and orientation are numpy arrays.
        """
        ee_pos, ee_orn = self.readEndEffectorState()
        ee_orn_mat = tools.getMatrixFromQuaternion(ee_orn)
        # Transform to the finger frame
        finger_pos = tools.array2vector(ee_pos) + ee_orn_mat @ tools.array2vector(self.ee_finger_offset)
        return tools.vector2array(finger_pos), np.array(ee_orn)


    def fingerErrorFlag(self, goal_pos, goal_orn):
        """
        goal_pos: (3, ) numpy array
        goal_orn: (4, ) numpy array
        Use the rotation angle between two rotation matrices to calculate the orn error.
        If the error of the finger is smaller than the threshold, error_bool=True.
        Return: pos_error_bool, orn_error_bool. 
        True if the error is bigger than the threshold (self.posErrorTolrance, self.ornErrorTolerance).
        """
        goal_pos = np.array(goal_pos)
        goal_orn = np.array(goal_orn)

        finger_pos, finger_orn = self.readFingerCenterState()
        pos_err = tools.frobenius_norm(np.array(finger_pos), np.array(goal_pos))
        orn_err = tools.exponential_angle_metric(tools.getMatrixFromQuaternion(finger_orn), tools.getMatrixFromQuaternion(goal_orn))

        if pos_err <= self.posErrorTolerance and orn_err <= self.ornErrorTolerance:
            return False 
        else:
            return True


    def jointErrorFlag(self, jointTargetState_list):
        """
        joint_target_state: a (8, ) array defining the target value of 8 controllable joints
        Return: False if the error is smaller than a threshold (self.jointErrorTolerance)
        """
        joint_error = 0.0
        jointTargetState_list = np.array(jointTargetState_list)

        for control_joint_idx,  control_joint_name in enumerate(self.robotControlJoints):
            joint_error += abs(self.readJointState(self.joints[control_joint_name].id)[0] \
                           - jointTargetState_list[control_joint_idx])

        if joint_error <= self.jointErrorTolerance:
            return False
        else:
            return True


    def jointLimitFilter(self, jointTargetState_list):
        """
        jointTargetState_list: a (8, ) array defining the target value of 8 controllable joints on the UR
        Return: a (8, ) array defining the target value within the allowable joints limit
        """
        
        # Only considering joint limit for the UR5 robot
        jointLimitTargetState_list = np.copy(np.array(jointTargetState_list))
        for joint_idx, jointTargetState in enumerate(jointTargetState_list[:-2]):
            if jointTargetState > math.pi:
                jointLimitTargetState_list[joint_idx] -= math.pi
            elif jointTargetState < -math.pi:
                jointLimitTargetState_list[joint_idx] += math.pi 

        return jointLimitTargetState_list

        
    def go_to(self, pos, orn):
        """
        pos: list of 3 floats
        orn: list of 4 floats, in quaternion
        Make the center of the finger tip reach a given target position in Cartesian world space.
        """
        # Transform to the ee frame
        ee_pos = tools.array2vector(pos) \
                - tools.getMatrixFromQuaternion(orn) @ tools.array2vector(self.ee_finger_offset) 
        ee_orn = orn
        
        # Retrun 8 values for the 8 controllable joints 
        jointTargetState_list = p.calculateInverseKinematics(self.robotID, self.eeID, targetPosition=ee_pos, 
                                                targetOrientation=orn, lowerLimits=self.jointsLowerLimit[:-2], 
                                                upperLimits=self.jointsUpperLimit[:-2], jointRanges=self.jointsRange[:-2], 
                                                restPoses=self.jointsRestPose[:-2])
        
        # TODO: Make the lower/higher limit work in the inverse kinematic! There are some problems with
        # the inverse kinematics as well. Some value cannot be satisfied

        error_flag = True
        simulationTimeStep = 0
        while error_flag:
            for jointIdx, jointName in enumerate(self.robotControlJoints):
                jointTargetState = jointTargetState_list[jointIdx]
                p.setJointMotorControl2(self.robotID, self.joints[jointName].id, p.POSITION_CONTROL,
                                        targetPosition=jointTargetState, force=self.joints[jointName].maxForce,
                                        maxVelocity=self.joints[jointName].maxVelocity/5) # keep the maxVelocity small to avoid overshoot in the PD control
            p.stepSimulation()
            time.sleep(1./240.)
            error_flag = self.jointErrorFlag(jointTargetState_list)

            simulationTimeStep += 1
            if simulationTimeStep >= self.maxControlTimeStep:
                break

        if self.fingerErrorFlag(pos, orn):
            raise ValueError('The goal pos and orn given are out of the workspace of the robot!')
        else:
            print('Finish moving to the goal pos and orn!')

    # TODO : Add measuring the gripper state (if it is not closed completely, then there is something grasped)
    def close_gripper(self):
        gripperMaxJointState = [self.joints[jointname].upperLimit for jointname in self.gripperControlJoints]
        for _ in range(self.gripperControlTimeStep):
            for jointIdx, jointName in enumerate(self.gripperControlJoints):
                jointTargetState = gripperMaxJointState[jointIdx]
                p.setJointMotorControl2(self.robotID, self.joints[jointName].id, p.POSITION_CONTROL,
                                        targetPosition=jointTargetState, force=self.joints[jointName].maxForce,
                                        maxVelocity=self.joints[jointName].maxVelocity/5)
            p.stepSimulation()
            time.sleep(1./240.)
        print('Gripper closed!')


    def open_gripper(self):
        gripperMaxJointState = [self.joints[jointname].lowerLimit for jointname in self.gripperControlJoints]
        for _ in range(self.gripperControlTimeStep):
            for jointIdx, jointName in enumerate(self.gripperControlJoints):
                jointTargetState = gripperMaxJointState[jointIdx]
                p.setJointMotorControl2(self.robotID, self.joints[jointName].id, p.POSITION_CONTROL,
                                        targetPosition=jointTargetState, force=self.joints[jointName].maxForce,
                                        maxVelocity=self.joints[jointName].maxVelocity/5)
            p.stepSimulation()
            time.sleep(1./240.)
        print('Gripper open!')


    def test(self, sim_timesteps=None):
        """
        sim_timesteps: if None, then simulate forever; if not, simulate for the sim_timesteps
        """
        if not sim_timesteps:
            while True:
                p.stepSimulation()
        else:
            for i in range(sim_timesteps):
                p.stepSimulation()
                time.sleep(1./240.)


    def debug(self):
        """
        Debug with the joint control window. Only the controllable joints are included.
        """
        userParams = dict()
        for name in self.gripperControlJoints:
            joint = self.joints[name]

            if name in self.initialJointValue.keys():
                userParam = p.addUserDebugParameter(name, joint.lowerLimit, joint.upperLimit, 
                                                    self.initialJointValue[name])
            else:
                userParam = p.addUserDebugParameter(name, joint.lowerLimit, joint.upperLimit, 0)
            userParams[name] = userParam
        while True:
            for name in self.gripperControlJoints:
                joint = self.joints[name]
                pose = p.readUserDebugParameter(userParams[name])
                p.setJointMotorControl2(self.robotID, joint.id, p.POSITION_CONTROL,
                                        targetPosition=pose, force=joint.maxForce, 
                                        maxVelocity=joint.maxVelocity)

            p.stepSimulation()


if __name__ == "__main__":

    # start simulation
    try:
        robotUrdfPath = "./urdf/imaginebot.urdf"
        rob = RobotWithGripper(robotUrdfPath)
        
        record_process = True
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
        target_finger_pos = np.array([0.6, 0.1, 0.1])
        target_finger_orn = np.array(p.getQuaternionFromEuler([0, math.pi / 2, 0])) 
        
        rob.go_to(target_finger_pos, target_finger_orn)
        
        rob.close_gripper()

        target_finger_pos = np.array([0.0, 0.6, 0.1])
        target_finger_orn = np.array(p.getQuaternionFromEuler([0, math.pi / 2, math.pi/2])) 

        rob.go_to(target_finger_pos, target_finger_orn)
        rob.open_gripper()

        rob.test(500)

        p.disconnect()
    except:
        p.disconnect()