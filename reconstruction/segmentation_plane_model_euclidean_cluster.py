"""
Segmentation out with plane model and euclidean cluster with PCL.
Plane model: http://pointclouds.org/documentation/tutorials/planar_segmentation.php#planar-segmentation
Euclidean cluster: http://pointclouds.org/documentation/tutorials/cluster_extraction.php#cluster-extraction

First use plane model to cut out all the plane. Then segement models with Euclidean cluster.
Python binding for Plane model: https://github.com/Sirokujira/python-pcl/blob/rc_patches4/examples/official/Segmentation/Plane_model_segmentation.py
Python binding for Euclidean cluster: https://github.com/Sirokujira/python-pcl/blob/rc_patches4/examples/official/Segmentation/cluster_extraction.py

@author: Hongtao Wu
Nov 25, 2019
"""

import pcl
import numpy as np
from plyfile import PlyData, PlyElement


def main(ply_file, ply_output_prefix):
    """
    Args:
    - ply_file: path to the ply file
    - ply_output_prefix: prefix for the output. (There can be multiple output)
    """

    # Load Point cloud
    cloud = pcl.PointCloud()
    plydata = PlyData.read(ply_file)
    ply_x = plydata['vertex']['x']
    ply_y = plydata['vertex']['y']
    ply_z = plydata['vertex']['z']
    
    points = np.zeros((ply_x.shape[0], 3), dtype=np.float32)
    points[:, 0] = ply_x
    points[:, 1] = ply_y
    points[:, 2] = ply_z

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

    # Save the point for each cluster
    for j, indices in enumerate(cluster_indices):
        num_verts = len(indices)
        verts_tuple = np.zeros((num_verts, ), dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4')])
        for i, indice in enumerate(indices):
            verts_tuple[i] = cloud_filtered[indice]

        el_verts = PlyElement.describe(verts_tuple, 'vertex')

        ply_new_data = PlyData([el_verts])
        ply_output_file = ply_output_prefix + '_%d.ply' % (j)
        ply_write = ply_new_data.write(ply_output_file)


if __name__ == "__main__":
    ply_file = 'tsdf-fusion/model/test_1124_aruco.ply'
    ply_output_prefix = 'tsdf-fusion/model/test_1124_aruco_remove_euclidean'
    main(ply_file, ply_output_prefix)