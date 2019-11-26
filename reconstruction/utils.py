#! /usr/bin/env python

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
    Return the AABB of the segmented object.

    Args:
    - points (nx3 float array): points from the tsdf
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