"""
Code for measuring the convexity of a 3D mesh.
Benchmarking for Open Containability Imagination.

Author: Hongtao Wu
July 02, 2020
"""

from __future__ import division
import trimesh
import os

def write_result(benchmark_dir, obj_name, convexity):
    concavity = 1 - convexity
    benchmark_result_path = os.path.join(benchmark_dir, obj_name + ".txt")
    
    with open(benchmark_result_path, 'w') as txt_file:
        writerow = 'container ' + str(concavity) + ' 0 1 2 3'
        txt_file.write(writerow)


def compute_convexity(obj_path):

    mesh = trimesh.load(obj_path)
    watertight = mesh.is_watertight
    
    # If the mesh is not watertight, the volume is garbage
    assert watertight == True 

    volume = mesh.volume
    ch_volume = mesh.convex_hull.volume

    convexity = volume / ch_volume

    return convexity


if __name__ == "__main__":
    data_folder  = "/home/hongtao/Dropbox/ICRA2021/data/test_set_all"
    benchmark_dir = "/home/hongtao/Dropbox/ICRA2021/benchmark/test_set_all_geometry_0703"
    obj_name_list  = os.listdir(data_folder)
    print "Object number: ", len(obj_name_list)
    
    for obj_name in obj_name_list:
        mesh_name = obj_name + "_mesh_0.obj"
        mesh_path = os.path.join(data_folder, obj_name, mesh_name)

        convexity = compute_convexity(mesh_path)
        write_result(benchmark_dir, obj_name, convexity)

        print obj_name
        print convexity
        print "======="


