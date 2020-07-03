"""
Code for measuring the convexity of a 3D mesh.
Benchmarking for Open Containability Imagination.

Author: Hongtao Wu
July 02, 2020
"""
from __future__ import division
import trimesh
import os


def convexity(obj_path):

    mesh = trimesh.load(obj_path)
    watertight = mesh.is_watertight
    # print "Watertight: ", watertight

    if not watertight:
        print obj_path
        print "Not watertight!"

    volume = mesh.volume
    ch_volume = mesh.convex_hull.volume

    convexity = volume / ch_volume

    print "======="


if __name__ == "__main__":
    data_folder  = "/home/hongtao/Dropbox/ICRA2021/data/test_set_all"
    obj_name_list  = os.listdir(data_folder)
    print "Object number: ", len(obj_name_list)
    
    for obj_name in obj_name_list:
        mesh_name = obj_name + "_mesh_0.obj"
        obj_path = os.path.join(data_folder, obj_name, mesh_name)

        convexity(obj_path)


