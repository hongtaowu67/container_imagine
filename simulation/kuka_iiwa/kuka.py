import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)

import pybullet as p
import numpy as np
import trimesh
import math
import pybullet_data
from time import sleep


class Kuka:

  def __init__(self, rootPath=None, timeStep=0.01):
    self.rootPath = rootPath
    self.timeStep = timeStep
    self.maxVelocity = .35
    self.maxForce = 200.
    self.fingerAForce = 2.5
    self.fingerBForce = 2.5
    self.fingerTipForce = 2.
    self.useInverseKinematics = 1
    self.useSimulation = 1
    self.useNullSpace = 21
    self.useOrientation = 1
    self.kukaEndEffectorIndex = 6
    self.kukaGripperIndex = 7
    #lower limits for null space
    self.ll = [-.967, -2.094, -2.967, -2.094, -2.967, -2.094, -3.054]
    #upper limits for null space
    self.ul = [.967,   2.094,  2.967,  2.094,  2.967,  2.094,  3.054]
    #joint ranges for null space
    self.jr = [5.8, 4, 5.8, 4, 5.8, 4, 6]
    #restposes for null space
    self.rp = [0, 0, 0, 0.5 * math.pi, 0, -math.pi * 0.5 * 0.66, 0]
    #joint damping coefficents
    self.jd = [
        0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001,
        0.00001, 0.00001, 0.00001, 0.00001
    ]

    self.reset()

    # Get Joint Info
    self.jointInfo = []
    for i in range(self.numJoints):
      self.jointInfo.append(p.getJointInfo(self.kukaUid, i))

    self.jointID = {'J0': 0,
                    'J1': 1,
                    'J2': 2,
                    'J3': 3,
                    'J4': 4,
                    'J5': 5,
                    'J6': 6,
                    'gripper_to_arm': 7,
                    'base_left_finger_joint': 8,
                    'left_finger_base_joint': 9, # fixed
                    'left_base_tip_joint': 10,
                    'base_right_giner_joint': 11,
                    'right_finger_base_joint': 12, # fixed
                    'right_base_tip_joint': 13}


  def reset(self):
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    objects = p.loadSDF(os.path.join(self.rootPath, "kuka_iiwa/object/robot/kuka_with_gripper2.sdf"))
    self.kukaUid = objects[0]

    p.resetBasePositionAndOrientation(self.kukaUid, [-0.100000, 0.000000, 0.10000],
                                      [0.000000, 0.000000, 0.000000, 1.000000])
    self.jointPositions = [
        0.006418, 0.413184, -0.011401, -1.589317, 0.005379, 1.137684, -0.006539, 
        0.000048, -0.299912, 0.000000, -0.000043, 0.299960, 0.000000, -0.000200
    ]

    self.numJoints = p.getNumJoints(self.kukaUid)
    for jointIndex in range(self.numJoints):
      p.resetJointState(self.kukaUid, jointIndex, self.jointPositions[jointIndex])
      p.setJointMotorControl2(self.kukaUid,
                              jointIndex,
                              p.POSITION_CONTROL,
                              targetPosition=self.jointPositions[jointIndex],
                              force=self.maxForce)

    # self.trayUid = p.loadURDF(os.path.join(self.rootPath, "kuka_iiwa/object/tray/tray.urdf"), 0.640000,
    #                           0.075000, 0.0, 0.000000, 0.000000, 1.000000, 0.000000)

    # Load plane
    p.loadURDF("plane.urdf", useFixedBase=True)
    self.endEffectorPos = [0.537, 0.0, 0.5]
    self.endEffectorAngle = 0

    self.motorNames = []
    self.motorIndices = []

    for i in range(self.numJoints):
      jointInfo = p.getJointInfo(self.kukaUid, i)
      qIndex = jointInfo[3]
      if qIndex > -1:
        #print("motorname")
        #print(jointInfo[1])
        self.motorNames.append(str(jointInfo[1]))
        self.motorIndices.append(i)


  def getActionDimension(self):
    if (self.useInverseKinematics):
      return len(self.motorIndices)
    return 6  #position x,y,z and roll/pitch/yaw euler angles of end effector


  def getObservationDimension(self):
    return len(self.getObservation())


  def getObservation(self):
    observation = []
    state = p.getLinkState(self.kukaUid, self.kukaGripperIndex)
    pos = state[0]
    orn = state[1]
    euler = p.getEulerFromQuaternion(orn)

    observation.extend(list(pos))
    observation.extend(list(euler))

    return observation


  def applyAction(self, motorCommands):

    #print ("self.numJoints")
    #print (self.numJoints)
    if (self.useInverseKinematics):

      dx = motorCommands[0]
      dy = motorCommands[1]
      dz = motorCommands[2]
      da = motorCommands[3]
      fingerAngle = motorCommands[4]

      state = p.getLinkState(self.kukaUid, self.kukaEndEffectorIndex)
      actualEndEffectorPos = state[0]
      #print("pos[2] (getLinkState(kukaEndEffectorIndex)")
      #print(actualEndEffectorPos[2])

      self.endEffectorPos[0] = self.endEffectorPos[0] + dx
      if (self.endEffectorPos[0] > 0.65):
        self.endEffectorPos[0] = 0.65
      if (self.endEffectorPos[0] < 0.50):
        self.endEffectorPos[0] = 0.50
      self.endEffectorPos[1] = self.endEffectorPos[1] + dy
      if (self.endEffectorPos[1] < -0.17):
        self.endEffectorPos[1] = -0.17
      if (self.endEffectorPos[1] > 0.22):
        self.endEffectorPos[1] = 0.22

      #print ("self.endEffectorPos[2]")
      #print (self.endEffectorPos[2])
      #print("actualEndEffectorPos[2]")
      #print(actualEndEffectorPos[2])
      #if (dz<0 or actualEndEffectorPos[2]<0.5):
      self.endEffectorPos[2] = self.endEffectorPos[2] + dz

      self.endEffectorAngle = self.endEffectorAngle + da
      pos = self.endEffectorPos
      # TODO: Need to add orientation for the end effector
      orn = p.getQuaternionFromEuler([0, -math.pi, 0])  # -math.pi,yaw])
      if (self.useNullSpace == 1):
        if (self.useOrientation == 1):
          jointPoses = p.calculateInverseKinematics(self.kukaUid, self.kukaEndEffectorIndex, pos,
                                                    orn, self.ll, self.ul, self.jr, self.rp)
        else:
          jointPoses = p.calculateInverseKinematics(self.kukaUid,
                                                    self.kukaEndEffectorIndex,
                                                    pos,
                                                    lowerLimits=self.ll,
                                                    upperLimits=self.ul,
                                                    jointRanges=self.jr,
                                                    restPoses=self.rp)
      else:
        if (self.useOrientation == 1):
          jointPoses = p.calculateInverseKinematics(self.kukaUid,
                                                    self.kukaEndEffectorIndex,
                                                    pos,
                                                    orn,
                                                    jointDamping=self.jd)
        else:
          jointPoses = p.calculateInverseKinematics(self.kukaUid, self.kukaEndEffectorIndex, pos)

      #print("jointPoses")
      #print(jointPoses)
      #print("self.kukaEndEffectorIndex")
      #print(self.kukaEndEffectorIndex)
      if (self.useSimulation):
        for i in range(self.kukaEndEffectorIndex + 1):
          #print(i)
          p.setJointMotorControl2(bodyUniqueId=self.kukaUid,
                                  jointIndex=i,
                                  controlMode=p.POSITION_CONTROL,
                                  targetPosition=jointPoses[i],
                                  targetVelocity=0,
                                  force=self.maxForce,
                                  maxVelocity=self.maxVelocity,
                                  positionGain=0.3,
                                  velocityGain=1)
      else:
        #reset the joint state (ignoring all dynamics, not recommended to use during simulation)
        for i in range(self.numJoints):
          p.resetJointState(self.kukaUid, i, jointPoses[i])
      #fingers
      p.setJointMotorControl2(self.kukaUid,
                              7,
                              p.POSITION_CONTROL,
                              targetPosition=self.endEffectorAngle,
                              force=self.maxForce)
      p.setJointMotorControl2(self.kukaUid,
                              8,
                              p.POSITION_CONTROL,
                              targetPosition=-fingerAngle,
                              force=self.fingerAForce)
      p.setJointMotorControl2(self.kukaUid,
                              11,
                              p.POSITION_CONTROL,
                              targetPosition=fingerAngle,
                              force=self.fingerBForce)

      p.setJointMotorControl2(self.kukaUid,
                              10,
                              p.POSITION_CONTROL,
                              targetPosition=0,
                              force=self.fingerTipForce)
      p.setJointMotorControl2(self.kukaUid,
                              13,
                              p.POSITION_CONTROL,
                              targetPosition=0,
                              force=self.fingerTipForce)

    else:
      for action in range(len(motorCommands)):
        motor = self.motorIndices[action]
        p.setJointMotorControl2(self.kukaUid,
                                motor,
                                p.POSITION_CONTROL,
                                targetPosition=motorCommands[action],
                                force=self.maxForce)
  

  def loadObject(self, urdfPath, startPos=[0, 0, 0], startOrn=[1, 0, 0, 0]):
    """
    urdfPath: path to the urdf file of the object.
    """
    p.loadURDF(urdfPath, startPos, startOrn)


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
              sleep(1./240.)

    
  def debug(self):
    """
    Debug with the joint control window. Only the controllable joints are included.
    """
    userParams = dict()
    for name, jointID in self.jointID.items():
      # Only revolute joint is presented
      if self.jointInfo[jointID][2] == 0:
        if jointID < 7:
          userParam = p.addUserDebugParameter(name, 
                                              self.ll[jointID], 
                                              self.ul[jointID], 
                                              self.jointPositions[jointID])
        else:
          userParam = p.addUserDebugParameter(name, 
                                              0, 
                                              0.5, 
                                              self.jointPositions[jointID])
      else:
        continue
      userParams[name] = userParam

    while True:
        for name in userParams.keys():
            jointID = self.jointID[name]
            pose = p.readUserDebugParameter(userParams[name])
            p.setJointMotorControl2(self.kukaUid, jointID, p.POSITION_CONTROL,
                                    targetPosition=pose, force=self.maxForce, 
                                    maxVelocity=self.maxVelocity)

        p.stepSimulation()


