import trimesh
import pybullet as p
import pybullet_data
import numpy as np

obj_vhacd_mesh = "/home/hongtao/Dropbox/ICRA2021/data/training_set/NonContainer/CVS_Pill_Bottle/CVS_Pill_Bottle_mesh_0_vhacd.obj"
obj_urdf = "/home/hongtao/Dropbox/ICRA2021/data/training_set/NonContainer/CVS_Pill_Bottle/CVS_Pill_Bottle_mesh_0.urdf"
content_urdf = "/home/hongtao/Dropbox/ICRA2021/data/general/sphere_mini.urdf" 

mesh = trimesh.load(obj_vhacd_mesh)

vol = mesh.volume

convex_hull_vol = mesh.convex_hull.volume

print "Volume: {}".format(vol)
print "Convex Hull Vol: {}".format(convex_hull_vol)

transform = mesh.bounding_box_oriented.primitive.transform
extends = mesh.bounding_box_oriented.primitive.extents

print "OBB center: \n{}".format(transform)
print "OBB extend: {}".format(extends)

center = transform[:3, -1]

# # Find the z-axis
# z_axis = np.array([0, 0, 1])
# z_axis_index = 0
# z_inner_product = -1
# for i in range(2):
#     inner_product = abs(np.dot(z_axis, transfrom[:3, i])
#     if inner_product > z_inner_product:


x_range = extends[0]/2
y_range = extends[1]/2
z_range = extends[2]/2

x_axis = transform[:3, 0]
y_axis = transform[:3, 1]
z_axis = transform[:3, 2]

vertices = np.zeros((8, 3))

vertices[0] = center + x_range * x_axis + y_range * y_axis + z_range * z_axis
vertices[1] = center - x_range * x_axis + y_range * y_axis + z_range * z_axis
vertices[2] = center + x_range * x_axis - y_range * y_axis + z_range * z_axis
vertices[3] = center - x_range * x_axis - y_range * y_axis + z_range * z_axis
vertices[4] = center + x_range * x_axis + y_range * y_axis - z_range * z_axis
vertices[5] = center - x_range * x_axis + y_range * y_axis - z_range * z_axis
vertices[6] = center + x_range * x_axis - y_range * y_axis - z_range * z_axis
vertices[7] = center - x_range * x_axis - y_range * y_axis - z_range * z_axis

print vertices

max_v = vertices.argmax(axis=0)
min_v = vertices.argmin(axis=0)

x_axis_compute = vertices[max_v[0]] - vertices[min_v[1]]
y_axis_compute = vertices[max_v[0]] - vertices[max_v[1]]

x_axis_compute = x_axis_compute[0:2]
y_axis_compute = y_axis_compute[0:2]

x_axis_range = np.linalg.norm(x_axis_compute)
y_axis_range = np.linalg.norm(y_axis_compute)

x_axis_compute = x_axis_compute / np.linalg.norm(x_axis_compute)
y_axis_compute = y_axis_compute / np.linalg.norm(y_axis_compute)

print "x_axis: {}".format(x_axis_compute)
print "y_axis: {}".format(y_axis_compute)

print "x_range: {}".format(x_axis_range)
print "y_range: {}".format(y_axis_range)

print "max: {}".format(max_v)
print "min: {}".format(min_v)

# p.connect(p.GUI)
# p.setAdditionalSearchPath(pybullet_data.getDataPath())

# p.loadURDF("plane.urdf")
# p.loadURDF(obj_urdf)

# p.loadURDF(content_urdf, basePosition=p1)
# p.loadURDF(content_urdf, basePosition=p2)
# p.loadURDF(content_urdf, basePosition=p3)
# p.loadURDF(content_urdf, basePosition=p4)
# p.loadURDF(content_urdf, basePosition=p5)
# p.loadURDF(content_urdf, basePosition=p6)
# p.loadURDF(content_urdf, basePosition=p7)
# p.loadURDF(content_urdf, basePosition=p8)


# import ipdb; ipdb.set_trace()

# p.disconnect()