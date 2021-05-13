# Utility functions for data processing
# Segmentation out with plane model and euclidean cluster with PCL.
# Plane model: http://pointclouds.org/documentation/tutorials/planar_segmentation.php#planar-segmentation
# Euclidean cluster: http://pointclouds.org/documentation/tutorials/cluster_extraction.php#cluster-extraction
# First use plane model to cut out all the plane. Then segement models with Euclidean cluster.
# Convert tsdf to ply source code: https://github.com/RobotLocomotion/spartan
# V-HACD repo: https://github.com/kmammou/v-hacd

# Author: Hongtao Wu
# Institution: Johns Hopkins University
# Date: Nov 05, 2019

from __future__ import print_function
import os
import time
import subprocess
import time
import array
import numpy as np
import pcl
from skimage import measure
from plyfile import PlyData, PlyElement
import trimesh
import csv
import os

#// --input camel.off --output camel_acd.wrl --log log.txt --resolution 1000000 --depth 20 --concavity 0.0025 --planeDownsampling 4 --convexhullDownsampling 4 --alpha 0.05 --beta 0.05 --gamma 0.00125
# --pca 0 --mode 0 --maxNumVerticesPerCH 256 --minVolumePerCH 0.0001 --convexhullApproximation 1 --oclDeviceID 2


def run_vhacd(vhacd_executable_dir,
              input_file,
              output_file,
              log='log.txt',
              resolution=500000,
              depth=20,
              concavity=0.0025,
              planeDownsampling=4,
              convexhullDownsampling=4,
              alpha=0.05,
              beta=0.05,
              gamma=0.00125,
              pca=0,
              mode=0,
              maxNumVerticesPerCH=256,
              minVolumePerCH=0.0001,
              convexhullApproximation=1):
    """
    The wrapper function to run the vhacd convex decomposition.
    """
    vhacd_executable = os.path.join(vhacd_executable_dir, "testVHACD")
    print("V-HACD executable: {}".format(vhacd_executable))
    if not os.path.isfile(vhacd_executable):
        print(vhacd_executable)
        raise ValueError('V-HACD executable not found, have you compiled it?')

    cmd = "cd %s && %s --input %s --output %s --log %s --resolution %s --depth %s --concavity %s --planeDownsampling %s --convexhullDownsampling %s --alpha %s --beta %s --gamma %s \
        --pca %s --mode %s --maxNumVerticesPerCH %s --minVolumePerCH %s --convexhullApproximation %s" % (
        vhacd_executable_dir, vhacd_executable, input_file, output_file, log,
        resolution, depth, concavity, planeDownsampling,
        convexhullDownsampling, alpha, beta, gamma, pca, mode,
        maxNumVerticesPerCH, minVolumePerCH, convexhullApproximation)

    print("cmd:\n", cmd)

    start_time = time.time()
    process = subprocess.Popen(cmd, shell=True)
    process.wait()
    elapsed = time.time() - start_time

    print("V-HACD took %d seconds" % (elapsed))


def write_urdf(urdf_path,
               obj_original_file,
               obj_vhacd_file,
               mass=0.0,
               origin_x=0.0,
               origin_y=0.0,
               origin_z=0.0,
               origin_roll=0.0,
               origin_pitch=0.0,
               origin_yaw=0.0,
               ixx=0.0,
               ixy=0.0,
               ixz=0.0,
               iyy=0.0,
               iyz=0.0,
               izz=0.0):
    """
    Writing the URDF file for Pybullet simulation.
    
    @type  obj_original_file: string
    @param obj_original_file: the file name of the original file, e.g., cup_0003.obj
    @type  obj_vhacd_file: string
    @param obj_vhacd_file: the filename of the vhacd file, e.g., cup_0003_vhacd.obj
    """

    f = open(urdf_path, 'w+')
    obj_name = obj_original_file.split('.')[0]

    f.write(
        '<?xml version=\"1.0\" ?>\n'
        '<robot name=\"%s.urdf\">\n'
        '  <link name=\"baseLink\">\n'
        '    <contact>\n'
        '      <lateral_friction value=\"1.0\"/>\n'
        '      <inertia_scaling value=\"1.0\"/>\n'
        '    </contact>\n'
        '    <inertial>\n'
        '      <origin rpy=\"0 0 0\" xyz=\"%.6f %.6f %.6f\"/>\n'
        '      <mass value=\"%.6f\"/>\n'
        '      <inertia ixx=\"%.6f\" ixy=\"%.6f\" ixz=\"%.6f\" iyy=\"%.6f\" iyz=\"%.6f\" izz=\"%.6f\"/>\n'
        '    </inertial>\n'
        '    <visual>\n'
        '      <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>\n'
        '      <geometry>\n'
        '       <mesh filename=\"%s\" scale=\"1 1 1\"/>\n'
        '      </geometry>\n'
        '       <material name=\"white\">\n'
        '        <color rgba=\"1 1 1 1\"/>\n'
        '      </material>\n'
        '    </visual>\n'
        '    <collision>\n'
        '      <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>\n'
        '      <geometry>\n'
        '        <mesh filename=\"%s\" scale=\"1 1 1\"/>\n'
        '      </geometry>\n'
        '    </collision>\n'
        '  </link>\n'
        '</robot>\n' % (obj_name, origin_x, origin_y, origin_z, mass, ixx, ixy,
                        ixz, iyy, iyz, izz, obj_original_file, obj_vhacd_file))

    f.close()


def segment_aabb(points, ply_output_prefix):
    """
    Return the AABB of the segmented object
    Trim off the plane and segment the object with Euclidean cluster
    Save the point cloud of the segmented objects

    Args:
    - points (nx3 float array): points from the tsdf
    - ply_output_prefix: prefix name for the output ply file, e.g., 'test_1124_aruco_3_mesh'

    Returns:
    - total_aabb: list of aabb of each object
    """

    # Load points into a point cloud class
    points = points.astype(np.float32)
    cloud = pcl.PointCloud()
    cloud.from_array(points)
    print('Point cloud data: ' + str(cloud.size) + ' points')

    # Filtered point
    cloud_filtered = pcl.PointCloud()
    cloud_filtered = cloud

    i = 0
    nr_points = cloud_filtered.size

    # Euclidean Cluster
    tree = cloud_filtered.make_kdtree()
    ec = cloud_filtered.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(0.015)
    ec.set_MinClusterSize(200)
    ec.set_MaxClusterSize(25000)
    ec.set_SearchMethod(tree)
    cluster_indices = ec.Extract()

    total_aabb = []

    # Compute AABB for each object
    # Save the point for each cluster
    for j, indices in enumerate(cluster_indices):
        # Save ply file
        num_verts = len(indices)
        obj_points = np.zeros((num_verts, 3))
        verts_tuple = np.zeros((num_verts, ),
                               dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4')])
        for i, indice in enumerate(indices):
            verts_tuple[i] = cloud_filtered[indice]
            obj_points[i, :] = cloud_filtered[indice]

        el_verts = PlyElement.describe(verts_tuple, 'vertex')

        ply_new_data = PlyData([el_verts])
        ply_output_file = ply_output_prefix + '_%d.ply' % (j)
        ply_write = ply_new_data.write(ply_output_file)

        # AABB
        max_x = np.max(obj_points[:, 0])
        min_x = np.min(obj_points[:, 0])
        max_y = np.max(obj_points[:, 1])
        min_y = np.min(obj_points[:, 1])
        max_z = np.max(obj_points[:, 2])
        min_z = np.min(obj_points[:, 2])
        obj_aabb = [(min_x, min_y, min_z), (max_x, max_y, max_z)]
        total_aabb.append(obj_aabb)
        print('object aabb: {}'.format(obj_aabb))

    return total_aabb


def convert_tsdf_to_ply(tsdf_bin_filename, tsdf_mesh_filename):
    """
    Converts the tsdf binary file to a mesh file in ply format
    The indexing in the tsdf is
    (x,y,z) <--> (x + y * dim_x + z * dim_x * dim_y)

    Args:
    - tsdf_bin_filename: binary file of the tsdf
    - tsdf_mesh_filename: output mesh from the tsdf

    Returns:
    - mesh_points: points on the mesh
    - tsdf: original tsdf from the binary file
    - voxelGridOrigin: Origin of the voxel
    - voxelSize: size of a voxel
    """
    start_time = time.time()
    fin = open(tsdf_bin_filename, "rb")

    tsdfHeader = array.array("f")  # f is the typecode for float32
    tsdfHeader.fromfile(fin, 8)
    fin.close()

    voxelGridDim = tsdfHeader[0:3]
    voxelGridDim = np.asarray(voxelGridDim, dtype=np.int)
    voxelGridOrigin = tsdfHeader[3:6]
    voxelSize = tsdfHeader[6]
    truncMargin = tsdfHeader[7]

    headerSize = 8
    tsdf_vec = np.fromfile(tsdf_bin_filename, np.float32)
    tsdf_vec = tsdf_vec[headerSize:]
    tsdf = np.reshape(tsdf_vec, voxelGridDim,
                      order='F')  # reshape using Fortran order

    verts, faces, normals, values = measure.marching_cubes_lewiner(
        tsdf, spacing=[voxelSize] * 3, level=0)

    # transform from voxel coordinates to camera coordinates
    # note x and y are flipped in the output of marching_cubes
    mesh_points = np.zeros_like(verts)
    mesh_points[:, 0] = voxelGridOrigin[0] + verts[:, 0]
    mesh_points[:, 1] = voxelGridOrigin[1] + verts[:, 1]
    mesh_points[:, 2] = voxelGridOrigin[2] + verts[:, 2]

    # Write the ply file
    ply_conversion_start_time = time.time()

    num_verts = verts.shape[0]
    num_faces = faces.shape[0]

    verts_tuple = np.zeros((num_verts, ),
                           dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4')])
    faces_tuple = np.zeros((num_faces, ),
                           dtype=[('vertex_indices', 'i4', (3, ))])

    for i in xrange(0, num_verts):
        verts_tuple[i] = tuple(mesh_points[i, :])

    for i in xrange(0, num_faces):
        faces_tuple[i] = faces[i, :].tolist()

    # Save the mesh
    el_verts = PlyElement.describe(verts_tuple, 'vertex')
    el_faces = PlyElement.describe(faces_tuple, 'face')

    ply_data = PlyData([el_verts, el_faces])
    print("Saving mesh to %s" % (tsdf_mesh_filename))
    ply = ply_data.write(tsdf_mesh_filename)

    print("Converting to ply format and writing to file took %d s" %
          (time.time() - start_time))

    # return mesh_points, tsdf, voxelGridOrigin, voxelSize


def ply2csv(ply_file, csv_file):
    """
    Convert ply file to csv. Each point is the vertices of an object.

    Args:
    - ply_file: path to the ply file
    - csv_file: path to save csv file
    """

    ply = PlyData.read(ply_file)

    vertices_num = len(ply['vertex']['x'])

    print("Number of all vertices: {}".format(vertices_num))

    with open(csv_file, 'w') as c_file:
        writer = csv.writer(c_file, quoting=csv.QUOTE_NONE)
        for i in range(vertices_num):
            writer.writerow([
                ply.elements[0].data['x'][i], ply.elements[0].data['y'][i],
                ply.elements[0].data['z'][i]
            ])

    print("Finished!")
