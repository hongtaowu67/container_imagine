"""
Utils functions

Segmentation out with plane model and euclidean cluster with PCL.
Plane model: http://pointclouds.org/documentation/tutorials/planar_segmentation.php#planar-segmentation
Euclidean cluster: http://pointclouds.org/documentation/tutorials/cluster_extraction.php#cluster-extraction

First use plane model to cut out all the plane. Then segement models with Euclidean cluster.
Python binding for Plane model: https://github.com/Sirokujira/python-pcl/blob/rc_patches4/examples/official/Segmentation/Plane_model_segmentation.py
Python binding for Euclidean cluster: https://github.com/Sirokujira/python-pcl/blob/rc_patches4/examples/official/Segmentation/cluster_extraction.py

Convert tsdf to ply source code: https://github.com/RobotLocomotion/spartan/blob/854b26e3af75910ef57b874db7853abd4249543e/src/catkin_projects/fusion_server/src/fusion_server/tsdf_fusion.py#L126

@author: Hongtao Wu
Nov 25, 2019
"""

import numpy as np
import pcl
from plyfile import PlyData, PlyElement


def quat2rotm(quat):
    """
    Quaternion to rotation matrix.
    
    Args:
    - quat (4, numpy array): quaternion w, x, y, z

    Returns:
    - rotm: (3x3 numpy array): rotation matrix
    """
    w = quat[0]
    x = quat[1]
    y = quat[2]
    z = quat[3]

    s = w*w + x*x + y*y + z*z

    rotm = np.array([[1-2*(y*y+z*z)/s, 2*(x*y-z*w)/s,   2*(x*z+y*w)/s  ],
                     [2*(x*y+z*w)/s,   1-2*(x*x+z*z)/s, 2*(y*z-x*w)/s  ],
                     [2*(x*z-y*w)/s,   2*(y*z+x*w)/s,   1-2*(x*x+y*y)/s]
    ])

    return rotm


def make_rigid_transformation(pos, orn):
    """
    Rigid transformation from position and orientation.

    Args:
    - pos (3, numpy array): translation
    - orn (4, numpy array): orientation in quaternion

    Returns:
    - homo_mat (4x4 numpy array): homogenenous transformation matrix
    """
    rotm = quat2rotm(orn)
    homo_mat = np.c_[rotm, np.reshape(pos, (3, 1))]
    homo_mat = np.r_[homo_mat, [[0, 0, 0, 1]]]
    
    return homo_mat


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

    # Plane model to filter the plane away
    seg = cloud.make_segmenter_normals(ksearch=50)
    seg.set_optimize_coefficients(True)
    seg.set_model_type(pcl.SACMODEL_NORMAL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    seg.set_distance_threshold(0.01)
    seg.set_normal_distance_weight(0.005)
    seg.set_max_iterations(500)
    plane_indices, plane_coefficients = seg.segment()

    # Filtered point
    cloud_filtered = pcl.PointCloud()
    cloud_filtered.from_array(np.delete(points, plane_indices, axis=0))

    i = 0
    nr_points = cloud_filtered.size
    
    # Euclidean Cluster
    tree = cloud_filtered.make_kdtree()
    ec = cloud_filtered.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(0.02)
    ec.set_MinClusterSize(100)
    ec.set_MaxClusterSize(25000)
    ec.set_SearchMethod(tree)
    cluster_indices = ec.Extract()

    print('cluster_indices : ' + str(cluster_indices.count) + " count.")

    total_aabb = []

    # Compute AABB for each object
    # Save the point for each cluster
    for j, indices in enumerate(cluster_indices):
        # Save ply file
        num_verts = len(indices)
        obj_points = np.zeros((num_verts, 3))
        verts_tuple = np.zeros((num_verts, ), dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4')])
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
        print 'object aabb: {}'.format(obj_aabb)

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

    # dim_x = voxelGridDim[0]
    # dim_y = voxelGridDim[1]
    # dim_z = voxelGridDim[2]


    headerSize = 8
    tsdf_vec = np.fromfile(tsdf_bin_filename, np.float32)
    tsdf_vec = tsdf_vec[headerSize:]
    tsdf = np.reshape(tsdf_vec, voxelGridDim, order='F') # reshape using Fortran order

    # print "tsdf.shape:", tsdf.shape
    # print "voxelGridDim: ", voxelGridDim
    # print "voxeGridOrigin: ", voxelGridOrigin
    # print "tsdf.shape:", tsdf.shape

    verts, faces, normals, values = measure.marching_cubes_lewiner(tsdf, spacing=[voxelSize]*3, level=0)


    # print "type(verts): ", type(verts)
    # print "verts.shape: ", verts.shape
    # print "faces.shape:", faces.shape

    # print "np.max(verts[:,0]): ", np.max(verts[:,0])
    # print "np.min(verts[:,0]): ", np.min(verts[:, 0])


    print "verts[0,:] = ", verts[0,:]
    print "faces[0,:] = ", faces[0,:]

    # transform from voxel coordinates to camera coordinates
    # note x and y are flipped in the output of marching_cubes
    mesh_points = np.zeros_like(verts)
    mesh_points[:,0] = voxelGridOrigin[0] + verts[:,0]
    mesh_points[:,1] = voxelGridOrigin[1] + verts[:,1]
    mesh_points[:,2] = voxelGridOrigin[2] + verts[:,2]

    # Write the ply file
    print "Converting numpy arrays to format for ply file"
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

    # Save the mesh
    el_verts = PlyElement.describe(verts_tuple, 'vertex')
    el_faces = PlyElement.describe(faces_tuple, 'face')

    ply_data = PlyData([el_verts, el_faces])
    print "Saving mesh to %s" %(tsdf_mesh_filename)
    ply = ply_data.write(tsdf_mesh_filename)

    print "Converting to ply format and writing to file took %d s" % (time.time() - start_time)

    return mesh_points, tsdf, voxelGridOrigin, voxelSize

