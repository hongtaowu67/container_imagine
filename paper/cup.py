import pybullet as p
import pybullet_data
import numpy as np

cup_urdf = "/home/hongtao/Dropbox/ICRA2021/data/general/cup/Cup_GeoCenter.urdf"

p.connect(p.GUI, options='--background_color_red=1.0 --background_color_green=1.0 --background_color_blue=1.0')
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
p.resetDebugVisualizerCamera(0.5, 0, 40, [0.0, 0.0, 0.0])

orn = [0, 0, 0]
p.loadURDF(cup_urdf, baseOrientation=p.getQuaternionFromEuler(orn))


p.stepSimulation()

import ipdb; ipdb.set_trace()


############################## pour_config ##############################
# # Load Bottle1
# self.bottle_id1 = p.loadURDF(self.bottle_urdf)
# p.changeDynamics(self.bottle_id1, -1, mass=1)

# p.setCollisionFilterPair(self.bottle_id, self.bottle_id1, -1, -1, 0)

# p.resetBasePositionAndOrientation(self.bottle_id1,
#         posObj=self.bottle_pos,
#         ornObj=p.getQuaternionFromEuler([0, 0, planar_angle]))

# bottle_constraint_Id1 = p.createConstraint(self.bottle_id1, -1, -1, -1, p.JOINT_FIXED, jointAxis=[0, 0, 0],
#         parentFramePosition=[bottle_half_length, 0.0, self.pour_pos[2]-self.bottle_pos[2]], 
#         childFramePosition=self.pour_pos,
#         parentFrameOrientation=p.getQuaternionFromEuler([0, 0, 0]),
#         childFrameOrientation=p.getQuaternionFromEuler([0, 0, planar_angle]))

# for i in range(self.simulation_iteration):
#     p.stepSimulation()

#     if self.check_process:
#         time.sleep(1. / 240.)           

#     orn = p.getQuaternionFromEuler([0, 1*np.pi/5 * math.sin(math.pi * 2 * (i) / int(4 * self.simulation_iteration)), planar_angle])
#     p.changeConstraint(bottle_constraint_Id1, pivot, jointChildFrameOrientation=orn, maxForce=50)

# # Load Bottle2
# self.bottle_id2 = p.loadURDF(self.bottle_urdf)
# # p.changeDynamics(self.bottle_id2, -1, mass=1)

# p.setCollisionFilterPair(self.bottle_id, self.bottle_id2, -1, -1, 0)
# p.setCollisionFilterPair(self.bottle_id1, self.bottle_id2, -1, -1, 0)

# p.resetBasePositionAndOrientation(self.bottle_id2,
#         posObj=self.bottle_pos,
#         ornObj=p.getQuaternionFromEuler([0, 0, planar_angle]))

# bottle_constraint_Id = p.createConstraint(self.bottle_id2, -1, -1, -1, p.JOINT_FIXED, jointAxis=[0, 0, 0],
#                         parentFramePosition=[bottle_half_length, 0.0, self.pour_pos[2]-self.bottle_pos[2]], 
#                         childFramePosition=self.pour_pos,
#                         parentFrameOrientation=p.getQuaternionFromEuler([0, 0, 0]),
#                         childFrameOrientation=p.getQuaternionFromEuler([0, 0, planar_angle]))

# self.set_content(planar_angle)

# import ipdb; ipdb.set_trace()
############################################################

