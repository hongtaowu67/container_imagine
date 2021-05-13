# A wrapper code for running TSDF fusion of several RGBD frame.
# After TSDF fusion, the code also segment out objects in the scene
# and save each one of them.

# TSDF fusion repo: https://github.com/hongtaowu67/TSDFfusion-cpu

# Author: Hongtao Wu
# Institution: Johns Hopkins University
# Date: Nov 23. 2019

from __future__ import print_function

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
from processing.utils import convert_tsdf_to_ply, segment_aabb


def run_tsdf_fusion(tsdf_fusion_dir,
                    data_dir,
                    camera_intrinsics_file,
                    voxel_grid_origin_x=0.0,
                    voxel_grid_origin_y=0.0,
                    voxel_grid_origin_z=0.0,
                    voxel_size=0.006,
                    voxel_grid_dim_x=500,
                    voxel_grid_dim_y=500,
                    voxel_grid_dim_z=500,
                    fast_tsdf_settings=False):
    """
    Simple wrapper to run the tsdf-fusion executable with the desired args
    For the tsdf fusion code, check out: https://github.com/hongtaowu67/TSDFfusion-cpu
    TSDF fusion will generate a binary file (.bin) and a point cloud file (.ply) and 
    save it in the data_dir.

    @type  tsdf_fusion_dir: string
    @param tsdf_fusion_dir: directory of the tsdf fusion package
    @type  data_dir: string
    @param data_dir: directory of the data being fused
    @type  camera_intrinsic_file: string
    @param camera_intrinsic_file: path to the intrinsic of the camera
    @type  voxel_grid_origin_x: float
    @param voxel_grid_origin_x: x coordinate of the origin for tsdf
    @type  voxel_grid_origin_y: float
    @param voxel_grid_origin_y: y coordinate of the origin for tsdf
    @type  voxel_grid_origin_z: float
    @param voxel_grid_origin_z: z coordinate of the origin for tsdf
    @type  voxel_size: float
    @param voxel_size: size of each voxel
    @type  voxel_grid_dim_x: int
    @param voxel_grid_dim_x: number of voxel in the x-axis
    @type  voxel_grid_dim_y: int
    @param voxel_grid_dim_y: number of voxel in the y-axis
    @type  voxel_grid_dim_z: int
    @param voxel_grid_dim_z: number of voxel in the z-axis
    @type  fast_tsdf_settings: bool
    @param  fast_tsdf_settings: whether to use fast tsdf params
    """
    # Path to the executable of the tsdf fusion
    tsdf_executable = os.path.join(tsdf_fusion_dir, 'build/tsdf-fusion-cpu')

    print("TSDF executable: {}".format(tsdf_executable))

    if not os.path.isfile(tsdf_executable):
        raise ValueError('tsdf executable not found, have you compiled it?')

    # Count frame number
    image_dir = os.path.join(data_dir, "rgbd")
    image_files = os.listdir(image_dir)
    frame_num = 0
    for f in image_files:
        if ".pose.txt" in f:
            frame_num += 1
    print("Frame number: {}".format(frame_num))

    # Write cmd
    cmd = "cd %s && %s %s %s" % (tsdf_fusion_dir, tsdf_executable,
                                 camera_intrinsics_file, image_dir)

    if fast_tsdf_settings:
        voxel_size = 0.004
        voxel_grid_dim_x = 120
        voxel_grid_dim_y = 100
        voxel_grid_dim_z = 80

    cmd += " " + str(frame_num)
    cmd += " " + str(voxel_size)
    cmd += " " + str(voxel_grid_dim_x)
    cmd += " " + str(voxel_grid_dim_y)
    cmd += " " + str(voxel_grid_dim_z)
    cmd += " " + str(voxel_grid_origin_x)
    cmd += " " + str(voxel_grid_origin_y)
    cmd += " " + str(voxel_grid_origin_z)

    print("cmd: {}".format(cmd))

    # Run TSDF fusion
    process = subprocess.Popen(cmd, shell=True)
    print("Started TSDF fusion subprocess, waiting for it to finish...")
    process.wait()

    # Move the bin and ply file to the image folder
    tsdf_bin_source = os.path.join(tsdf_fusion_dir, 'tsdf.bin')
    tsdf_ply_source = os.path.join(tsdf_fusion_dir, 'tsdf.ply')

    tsdf_result_dir = os.path.join(data_dir, "rgbd")
    if os.path.exists(tsdf_result_dir):
        pass
    else:
        os.mkdir(tsdf_result_dir)
    tsdf_bin_dest = os.path.join(tsdf_result_dir, 'tsdf.bin')
    tsdf_ply_dest = os.path.join(tsdf_result_dir, 'tsdf.ply')

    shutil.move(tsdf_bin_source, tsdf_bin_dest)
    shutil.move(tsdf_ply_source, tsdf_ply_dest)
    print("Finish tsdf fusion: {}".format(tsdf_ply_dest))


def tsdf_fusion_postprocess(tsdf_bin_file,
                            tsdf_ply_file,
                            ply_output_prefix,
                            mesh_output_prefix,
                            mesh_output_type='obj'):
    """
    Post-processing after the tsdf fusion
    1. Convert tsdf to mesh (ply)
    2. Segment the scene into individual objects

    This function will output the point clouds (.ply) and meshes 
    (.ply/.obj) of a list of segmented object from the scene.

    Converts the tsdf binary file to a mesh file in ply format
    The indexing in the tsdf is
    (x,y,z) <--> (x + y * dim_x + z * dim_x * dim_y)

    @type  tsdf_bin_file: string
    @param tsdf_bin_file: path to the tsdf binary file (.bin)
    @type  tsdf_ply_file: string
    @param tsdf_ply_file: path to the tsdf point cloud file (.ply)
    @type  ply_output_prefix: string
    @param ply_output_prefix: prefix of the output point cloud file
    @type  mesh_output_prefix: string
    @param mesh_output_prefix: prefix of the output mesh file
    @type  mesh_output_type: string
    @param mesh_output_type: type of the output mesh -- ply / obj
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
    tsdf = np.reshape(tsdf_vec, voxelGridDim,
                      order='F')  # reshape using Fortran order

    # Segment the mesh points
    objects_aabb = segment_aabb(mesh_points, ply_output_prefix)

    for obj_id, obj_aabb in enumerate(objects_aabb):
        min_x_in_voxel = max(
            int((obj_aabb[0][0] - voxelGridOrigin[0]) / voxelSize) - 3, 0)
        max_x_in_voxel = min(
            int((obj_aabb[1][0] - voxelGridOrigin[0]) / voxelSize) + 3,
            voxelGridDim[0])
        min_y_in_voxel = max(
            int((obj_aabb[0][1] - voxelGridOrigin[1]) / voxelSize) - 2, 0)
        max_y_in_voxel = min(
            int((obj_aabb[1][1] - voxelGridOrigin[1]) / voxelSize) + 10,
            voxelGridDim[1])
        min_z_in_voxel = max(
            int((obj_aabb[0][2] - voxelGridOrigin[2]) / voxelSize) - 3, 0)
        max_z_in_voxel = min(
            int((obj_aabb[1][2] - voxelGridOrigin[2]) / voxelSize) + 3,
            voxelGridDim[2])

        # Crop the tsdf
        obj_tsdf = tsdf[min_x_in_voxel:max_x_in_voxel,
                        min_y_in_voxel:max_y_in_voxel,
                        min_z_in_voxel:max_z_in_voxel]
        # Reconstruct with marching cube
        obj_verts, obj_faces, obj_normals, obj_values = measure.marching_cubes_lewiner(
            obj_tsdf, spacing=[voxelSize] * 3, level=0)

        obj_mesh_points = np.zeros_like(obj_verts)
        obj_mesh_points[:, 0] = voxelGridOrigin[
            0] + obj_verts[:, 0] + min_x_in_voxel * voxelSize
        obj_mesh_points[:, 1] = voxelGridOrigin[
            1] + obj_verts[:, 1] + min_y_in_voxel * voxelSize
        obj_mesh_points[:, 2] = voxelGridOrigin[
            2] + obj_verts[:, 2] + min_z_in_voxel * voxelSize

        obj_num_verts = obj_verts.shape[0]
        obj_num_faces = obj_faces.shape[0]

        # Save PLY file
        if mesh_output_type == 'ply':
            obj_verts_tuple = np.zeros((obj_num_verts, ),
                                       dtype=[('x', 'f4'), ('y', 'f4'),
                                              ('z', 'f4')])
            obj_faces_tuple = np.zeros((obj_num_faces, ),
                                       dtype=[('vertex_indices', 'i4', (3, ))])

            for i in xrange(0, obj_num_verts):
                obj_verts_tuple[i] = tuple(obj_mesh_points[i, :])

            for i in xrange(0, obj_num_faces):
                obj_faces_tuple[i] = obj_faces[i, :].tolist()

            obj_el_verts = PlyElement.describe(obj_verts_tuple, 'vertex')
            obj_el_faces = PlyElement.describe(obj_faces_tuple, 'face')

            obj_ply_data = PlyData([obj_el_verts, obj_el_faces])
            obj_mesh_filename = mesh_output_prefix + '_%d.ply' % (obj_id)
            obj_ply = obj_ply_data.write(obj_mesh_filename)

        # Save OBJ file
        if mesh_output_type == 'obj':
            obj_mesh_filename = mesh_output_prefix + '_%d.obj' % (obj_id)
            f = open(obj_mesh_filename, 'w')
            for obj_mesh_point in obj_mesh_points:
                f.write('v {0} {1} {2}\n'.format(obj_mesh_point[0],
                                                 obj_mesh_point[1],
                                                 obj_mesh_point[2]))
            for item in obj_normals:
                f.write('vn {0} {1} {2}\n'.format(item[0], item[1], item[2]))
            obj_faces += 1
            for item in obj_faces:
                f.write('f {0}//{0} {1}//{1} {2}//{2}\n'.format(
                    item[0], item[1], item[2]))
            f.close()
