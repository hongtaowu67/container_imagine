"""
Find the geometric center of a ply file and centralize the object.

June 15, 2020
"""
from __future__ import division
from plyfile import PlyData, PlyElement
import numpy as np

ply_file_path = "/home/hongtao/Dropbox/ICRA2021/data/general/cup/Cup.ply"
output_ply_file_path = "/home/hongtao/Dropbox/ICRA2021/data/general/cup/Cup_GeoCenter.ply"

plydata = PlyData.read(ply_file_path)

vertex_num = plydata.elements[0].count

min_x = 10000
max_x = -10000
min_y = 10000
max_y = -10000
min_z = 10000
max_z = -10000

for i in range(vertex_num):    
    x, y, z = plydata['vertex'][i]
    
    if x > max_x:
        max_x = x
    if x < min_x:
        min_x = x
    if y > max_y:
        max_y = y
    if y < min_y:
        min_y = y
    if z > max_z:
        max_z = z
    if z < min_z:
        min_z = z

center_x = (min_x + max_x) / 2
center_y = (min_y + max_y) / 2
center_z = (min_z + max_z) / 2

print "Center: {}, {}, {}".format(center_x, center_y, center_z)
print "Scale: {}, {}, {}". format(max_x - min_x, max_y - min_y, max_z - min_z)

plydata['vertex']['x'] -= center_x
plydata['vertex']['y'] -= center_y
plydata['vertex']['z'] -= center_z

# Rotate the cup by -np.pi/2

for i in range(vertex_num):
    x, y, z = plydata['vertex'][i]
    plydata['vertex'][i] = (y, -x, z)

plydata.write(output_ply_file_path)

