"""
Code for measuring the convexity of a 3D mesh.
Benchmarking for Open Containability Imagination.

Author: Hongtao Wu
July 02, 2020
"""
from __future__ import division
import trimesh
import os
from pymeshfix._meshfix import PyTMesh


def convexity(obj_path):

    mesh = trimesh.load(obj_path)
    watertight = mesh.is_watertight
    # print "Watertight: ", watertight

    if not watertight:
        print obj_path
        print "Not watertight!"

        # watertight_filename = obj_path.split(".")[0] + "_wt.ply"
        # print watertight_filename
        # mfix = PyTMesh()
        # mfix.load_file(obj_path)
        # mfix.fill_small_boundaries(nbe=40, refine=True)
        # mfix.clean(max_iters=10, inner_loops=3)
        # mfix.save_file(watertight_filename)
        
        # repaired_mesh = trimesh.load(watertight_filename)
        # reparied_watertight = repaired_mesh.is_watertight
        # print "Repaired watertight: ", reparied_watertight

    
    # volume = mesh.volume
    # ch_volume = mesh.convex_hull.volume

    # print "Volume: ", volume
    # print "CH Volume: ", ch_volume

    # convexity = volume / ch_volume

    # print "======="


if __name__ == "__main__":
    data_folder  = "/home/hongtao/Dropbox/ICRA2021/data/test_set_all"
    obj_name_list  = os.listdir(data_folder)
    # obj_name_list = ["Amazon_Hammer"]
    for obj_name in obj_name_list:
        mesh_name = obj_name + "_mesh_0.obj"
        # mesh_vhacd_name = obj_name + "_mesh_0_vhacd.obj"
        obj_path = os.path.join(data_folder, obj_name, mesh_name)
        # obj_vhacd_path = os.path.join(data_folder, obj_name, mesh_vhacd_name)

        convexity(obj_path)


