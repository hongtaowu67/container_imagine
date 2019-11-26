#!/usr/bin/env python
"""
A wrapper code for running TSDF fusion of several RGBD frame. The base frame is at the ArUco tag.
TSDF fusion sourced from: https://github.com/andyzeng/tsdf-fusion
General code sourced from: https://github.com/RobotLocomotion/spartan/blob/854b26e3af75910ef57b874db7853abd4249543e/src/catkin_projects/fusion_server/src/fusion_server/tsdf_fusion.py#L126
@ author: Hongtao Wu
Nov 23. 2019
"""

import os
import subprocess
import shutil
import numpy as np
import math
import yaml
import time
from skimage import measure
from plyfile import PlyData, PlyElement
import array


def run_tsdf_fusion_cuda(tsdf_fusion_dir, image_folder, camera_intrinsics_file, output_dir=None, voxel_grid_origin_x=0.0,
    voxel_grid_origin_y=0.0, voxel_grid_origin_z=0.0, voxel_size=0.006,
    voxel_grid_dim_x=500, voxel_grid_dim_y=500, voxel_grid_dim_z=500, fast_tsdf_settings=False):
    """
    Simple wrapper to call the tsdf-fusion executable with the desired args
    """
    if output_dir is None:
        output_dir = os.path.dirname(image_folder)
        print "output_dir: ", output_dir

    tsdf_executable = os.path.join(tsdf_fusion_dir, 'demo_aruco') # The base frame is at the ArUco tag
    if not os.path.isfile(tsdf_executable):
        raise ValueError('tsdf executable not found, have you compiled it?')

    cmd = "cd %s && %s %s %s" %(tsdf_fusion_dir, tsdf_executable,  camera_intrinsics_file, image_folder)


    if fast_tsdf_settings:
        voxel_size = 0.004
        voxel_grid_dim_x = 180
        voxel_grid_dim_y = 100
        voxel_grid_dim_z = 100
        
    
    cmd += " " + str(voxel_size)
    cmd += " " + str(voxel_grid_dim_x)
    cmd += " " + str(voxel_grid_dim_y)
    cmd += " " + str(voxel_grid_dim_z)

    cmd += " " + str(voxel_grid_origin_x)
    cmd += " " + str(voxel_grid_origin_y)
    cmd += " " + str(voxel_grid_origin_z)

    print "cmd:\n", cmd

    start_time = time.time()
    process = subprocess.Popen(cmd, shell=True)
    print "started subprocess, waiting for it to finish"
    process.wait()
    elapsed = time.time() - start_time

    tsdf_bin = os.path.join(image_folder, 'tsdf.bin')
    tsdf_ply = os.path.join(image_folder, 'tsdf.ply')

    # tsdf_bin_new = os.path.join(output_dir, 'tsdf.bin')
    # tsdf_ply_new = os.path.join(output_dir, 'fusion_pointcloud.ply')
    
    # shutil.move(tsdf_bin, tsdf_bin_new)
    # shutil.move(tsdf_ply, tsdf_ply_new)

    print "tsdf-fusion took %d seconds" %(elapsed)



def convert_tsdf_to_ply(tsdf_bin_filename, tsdf_mesh_filename):
    """
    Converts the tsdf binary file to a mesh file in ply format
    The indexing in the tsdf is
    (x,y,z) <--> (x + y * dim_x + z * dim_x * dim_y)
    """
    start_time = time.time()
    fin = open(tsdf_bin_filename, "rb")

    tsdfHeader = array.array("f")  # f is the typecode for float32
    tsdfHeader.fromfile(fin, 8)
    # print tsdfHeader
    # print type(tsdfHeader)

    voxelGridDim = tsdfHeader[0:3]
    voxelGridDim = np.asarray(voxelGridDim, dtype=np.int)
    voxelGridOrigin = tsdfHeader[3:6]
    voxelSize = tsdfHeader[6]
    truncMargin = tsdfHeader[7]

    dim_x = voxelGridDim[0]
    dim_y = voxelGridDim[1]
    dim_z = voxelGridDim[2]


    headerSize = 8
    tsdf_vec = np.fromfile(tsdf_bin_filename, np.float32)
    tsdf_vec = tsdf_vec[headerSize:]
    tsdf = np.reshape(tsdf_vec, voxelGridDim, order='F') # reshape using Fortran order

    # for loop version of the above reshape operation
    # for x in xrange(0, dim_x):
    #     for y in xrange(0, dim_y):
    #         for z in xrange(0, dim_z):
    #             idx = x + y * dim_x + z * dim_x * dim_y
    #             tsdf[x,y,z] = tsdf_vec[idx]


    print "tsdf.shape:", tsdf.shape
    print "voxelGridDim: ", voxelGridDim
    print "voxeGridOrigin: ", voxelGridOrigin
    print "tsdf.shape:", tsdf.shape

    verts, faces, normals, values = measure.marching_cubes_lewiner(tsdf, spacing=[voxelSize]*3, level=0)


    print "type(verts): ", type(verts)
    print "verts.shape: ", verts.shape
    print "faces.shape:", faces.shape

    print "np.max(verts[:,0]): ", np.max(verts[:,0])
    print "np.min(verts[:,0]): ", np.min(verts[:, 0])


    print "verts[0,:] = ", verts[0,:]
    print "faces[0,:] = ", faces[0,:]

    # transform from voxel coordinates to camera coordinates
    # note x and y are flipped in the output of marching_cubes
    mesh_points = np.zeros_like(verts)
    # mesh_points = verts
    mesh_points[:,0] = voxelGridOrigin[0] + verts[:,0]
    mesh_points[:,1] = voxelGridOrigin[1] + verts[:,1]
    mesh_points[:,2] = voxelGridOrigin[2] + verts[:,2]

    # permute faces to get visualization
    # faces = np.flip(faces, 1)


    # try writing to the ply file
    print "converting numpy arrays to format for ply file"
    ply_conversion_start_time = time.time()

    num_verts = verts.shape[0]
    num_faces = faces.shape[0]

    verts_tuple = np.zeros((num_verts,), dtype=[('x', 'f4'), ('y', 'f4'),
                                                ('z', 'f4')])
    faces_tuple = np.zeros((num_faces,), dtype=[('vertex_indices', 'i4', (3,))])

    for i in xrange(0, num_verts):
        verts_tuple[i] = tuple(mesh_points[i, :])

    for i in xrange(0, num_faces):
        faces_tuple[i] = faces[i, :].tolist()



    # save it out
    # try to save it
    el_verts = PlyElement.describe(verts_tuple, 'vertex')
    el_faces = PlyElement.describe(faces_tuple, 'face')

    ply_data = PlyData([el_verts, el_faces])
    print "saving mesh to %s" %(tsdf_mesh_filename)
    ply = ply_data.write(tsdf_mesh_filename)

    print "converting to ply format and writing to file took %d s" % (time.time() - start_time)

# Test
if __name__ == "__main__":
    tsdf_fusion_dir = "/home/hongtao/src/cup_imagine/reconstruction/tsdf-fusion"
    image_folder = os.path.join(tsdf_fusion_dir, "data/tsdf_data/rgbd-frames")
    camera_intrinsics_file = os.path.join(tsdf_fusion_dir, "data/tsdf_data/camera-intrinsics.txt")
    run_tsdf_fusion_cuda(tsdf_fusion_dir, image_folder, camera_intrinsics_file, 
        voxel_grid_origin_x=-0.3, voxel_grid_origin_y=0.005, voxel_grid_origin_z=-0.13, fast_tsdf_settings=True)

    tsdf_bin_filename = os.path.join(tsdf_fusion_dir, 'model/tsdf.bin')
    tsdf_mesh_filename = os.path.join(tsdf_fusion_dir, 'model/test_1124_aruco.ply')
    convert_tsdf_to_ply(tsdf_bin_filename, tsdf_mesh_filename)
