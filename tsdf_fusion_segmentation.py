#!/usr/bin/env python
"""
A wrapper code for running TSDF fusion of several RGBD frame. The base frame is at the ArUco tag.
After TSDF fusion, the code also segment out the object and save each one of the object.
TSDF fusion sourced from: https://github.com/andyzeng/tsdf-fusion

@author: Hongtao Wu
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
from reconstruction.utils import segment_aabb, convert_tsdf_to_ply


def run_tsdf_fusion_cuda(tsdf_fusion_dir, image_folder, camera_intrinsics_file, output_dir=None, voxel_grid_origin_x=0.0,
    voxel_grid_origin_y=0.0, voxel_grid_origin_z=0.0, voxel_size=0.006,
    voxel_grid_dim_x=500, voxel_grid_dim_y=500, voxel_grid_dim_z=500, fast_tsdf_settings=False):
    """
    Simple wrapper to call the tsdf-fusion executable with the desired args
    """
    if output_dir is None:
        output_dir = os.path.dirname(image_folder)
        print "output_dir: ", output_dir
    
    # TODO: if GPU is available, use GPU version of tsdf fusion
    tsdf_executable = os.path.join(tsdf_fusion_dir, 'tsdf-fusion-cpu-24') # The base frame is at the ArUco tag, 20 frames
    if not os.path.isfile(tsdf_executable):
        raise ValueError('tsdf executable not found, have you compiled it?')

    cmd = "cd %s && %s %s %s" %(tsdf_fusion_dir, tsdf_executable,  camera_intrinsics_file, image_folder)

    # Assume the object is no bigger than 0.2m x 0.2m
    if fast_tsdf_settings:
        voxel_size = 0.004
        voxel_grid_dim_x = 80
        voxel_grid_dim_y = 80
        voxel_grid_dim_z = 60
        
    
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

    # Move the bin and ply file to the image folder
    tsdf_bin_source = os.path.join(tsdf_fusion_dir, 'model/tsdf.bin')
    tsdf_ply_source = os.path.join(tsdf_fusion_dir, 'model/tsdf.ply')
    tsdf_bin_dest = os.path.join(image_folder, 'tsdf.bin')
    tsdf_ply_dest = os.path.join(image_folder, 'tsdf.ply')

    shutil.move(tsdf_bin_source, tsdf_bin_dest)
    shutil.move(tsdf_ply_source, tsdf_ply_dest)

    print "tsdf-fusion took %d seconds" %(elapsed)



def segment_tsdf_fast(tsdf_bin_file, tsdf_ply_file, ply_output_prefix, obj_mesh_output_prefix, output_type='obj', tsdf_mesh_file=None):
    """
    1. Convert tsdf to mesh (ply)
    2. Segment the mesh into individual objects
    Converts the tsdf binary file to a mesh file in ply format
    The indexing in the tsdf is
    (x,y,z) <--> (x + y * dim_x + z * dim_x * dim_y)
    """

    plydata = PlyData.read(tsdf_ply_file)
    ply_x = plydata['vertex']['x']
    ply_y = plydata['vertex']['y']
    ply_z = plydata['vertex']['z']
    mesh_points = np.zeros((ply_x.shape[0], 3), dtype=np.float32)
    mesh_points[:, 0] = ply_x
    mesh_points[:, 1] = ply_y
    mesh_points[:, 2] = ply_z

    fin = open(tsdf_bin_file, "rb")

    tsdfHeader = array.array("f")  # f is the typecode for float32
    tsdfHeader.fromfile(fin, 8)
    fin.close()

    voxelGridOrigin = tsdfHeader[3:6]
    voxelSize = tsdfHeader[6]
    voxelGridDim = tsdfHeader[0:3]
    voxelGridDim = np.asarray(voxelGridDim, dtype=np.int)

    headerSize = 8
    tsdf_vec = np.fromfile(tsdf_bin_file, np.float32)
    tsdf_vec = tsdf_vec[headerSize:]
    tsdf = np.reshape(tsdf_vec, voxelGridDim, order='F') # reshape using Fortran order

    if tsdf_mesh_file is not None:
        # Save the reconstruction of the whole scene
        print("Saving the full reconstruction of the scene...")
        convert_tsdf_to_ply(tsdf_bin_file, tsdf_mesh_file)

    # Segment the mesh points
    objects_aabb = segment_aabb(mesh_points, ply_output_prefix)
    
    for obj_id, obj_aabb in enumerate(objects_aabb):
        min_x_in_voxel = max(int((obj_aabb[0][0] - voxelGridOrigin[0]) / voxelSize) - 3, 0)
        max_x_in_voxel = min(int((obj_aabb[1][0] - voxelGridOrigin[0]) / voxelSize) + 3, voxelGridDim[0])
        min_y_in_voxel = max(int((obj_aabb[0][1] - voxelGridOrigin[1]) / voxelSize) - 2, 0)
        max_y_in_voxel = min(int((obj_aabb[1][1] - voxelGridOrigin[1]) / voxelSize) + 10, voxelGridDim[1])
        min_z_in_voxel = max(int((obj_aabb[0][2] - voxelGridOrigin[2]) / voxelSize) - 3, 0)
        max_z_in_voxel = min(int((obj_aabb[1][2] - voxelGridOrigin[2]) / voxelSize) + 3, voxelGridDim[2])

        # Crop the tsdf
        obj_tsdf = tsdf[min_x_in_voxel:max_x_in_voxel, min_y_in_voxel:max_y_in_voxel, min_z_in_voxel:max_z_in_voxel]
        # Reconstruct with marching cube
        obj_verts, obj_faces, obj_normals, obj_values = measure.marching_cubes_lewiner(obj_tsdf, spacing=[voxelSize]*3, level=0)

        obj_mesh_points = np.zeros_like(obj_verts)
        obj_mesh_points[:,0] = voxelGridOrigin[0] + obj_verts[:,0] + min_x_in_voxel * voxelSize
        obj_mesh_points[:,1] = voxelGridOrigin[1] + obj_verts[:,1] + min_y_in_voxel * voxelSize
        obj_mesh_points[:,2] = voxelGridOrigin[2] + obj_verts[:,2] + min_z_in_voxel * voxelSize

        obj_num_verts = obj_verts.shape[0]
        obj_num_faces = obj_faces.shape[0]
        
        # Save PLY file
        if output_type == 'ply':
            obj_verts_tuple = np.zeros((obj_num_verts,), dtype=[('x', 'f4'), ('y', 'f4'),
                                                        ('z', 'f4')])
            obj_faces_tuple = np.zeros((obj_num_faces,), dtype=[('vertex_indices', 'i4', (3,))])

            for i in xrange(0, obj_num_verts):
                obj_verts_tuple[i] = tuple(obj_mesh_points[i, :])

            for i in xrange(0, obj_num_faces):
                obj_faces_tuple[i] = obj_faces[i, :].tolist()

            obj_el_verts = PlyElement.describe(obj_verts_tuple, 'vertex')
            obj_el_faces = PlyElement.describe(obj_faces_tuple, 'face')

            obj_ply_data = PlyData([obj_el_verts, obj_el_faces])
            obj_mesh_filename = obj_mesh_output_prefix + '_%d.ply' % (obj_id)
            obj_ply = obj_ply_data.write(obj_mesh_filename)

        # Save OBJ file
        if output_type == 'obj':
            obj_mesh_filename = obj_mesh_output_prefix + '_%d.obj' % (obj_id)
            f = open(obj_mesh_filename, 'w')
            for obj_mesh_point in obj_mesh_points:
                f.write('v {0} {1} {2}\n'.format(obj_mesh_point[0], obj_mesh_point[1], obj_mesh_point[2]))
            for item in obj_normals:
                f.write('vn {0} {1} {2}\n'.format(item[0], item[1], item[2]))
            obj_faces += 1
            for item in obj_faces:
                f.write('f {0}//{0} {1}//{1} {2}//{2}\n'.format(item[0], item[1], item[2]))
            f.close()




# Test
if __name__ == "__main__":
<<<<<<< HEAD:reconstruction/tsdf_fusion_segmentation.py
    tsdf_fusion_dir = "/home/hongtao/src/cup_imagine/reconstruction/tsdf-fusion"
    model_output_dir = '/home/hongtao/src/cup_imagine/model'
    object_name = '1209_smalltape'
    image_folder = os.path.join(tsdf_fusion_dir, "data/tsdf_data", object_name, "rgbd-frames")
    camera_intrinsics_file = os.path.join(tsdf_fusion_dir, "data/tsdf_data", object_name, "camera-intrinsics.txt")
=======
    root_dir = os.getcwd()
    tsdf_fusion_dir = os.path.join(root_dir, 'reconstruction/tsdf-fusion')

    model_name = '19-12-26'
    model_output_dir = os.path.join(root_dir, 'data', model_name)
    image_folder = os.path.join(root_dir, 'data', model_name, 'rgbd')
    camera_intrinsics_file = os.path.join(root_dir, "calibrate/camera-intrinsics.txt")
    
>>>>>>> 595ddfbe3c14d371f3552735540010c46523dfca:tsdf_fusion_segmentation.py
    run_tsdf_fusion_cuda(tsdf_fusion_dir, image_folder, camera_intrinsics_file, 
        voxel_grid_origin_x=-0.2, voxel_grid_origin_y=-0.5, voxel_grid_origin_z=0.0, fast_tsdf_settings=True)

    tsdf_bin_file = os.path.join(root_dir, 'data', model_name, 'rgbd/tsdf.bin')
    # tsdf_mesh_file = os.path.join(model_output_dir, object_name, object_name + '_total.ply')
    tsdf_ply_file = os.path.join(root_dir, 'data', model_name, 'rgbd/tsdf.ply')
    ply_output_prefix = os.path.join(root_dir, 'data', model_name, model_name + '_point_debug')
    obj_mesh_output_prefix = os.path.join(root_dir, 'data', model_name, model_name + '_mesh_debug')
    # segment_tsdf(tsdf_bin_file, tsdf_mesh_file, ply_output_prefix, obj_mesh_output_prefix)
    segment_tsdf_fast(tsdf_bin_file, tsdf_ply_file, ply_output_prefix, obj_mesh_output_prefix)
