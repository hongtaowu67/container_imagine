import pcl
import numpy as np
import random
from plyfile import PlyData, PlyElement


def main(ply_file, ply_output_file):
    #   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    #
    #   // Fill in the cloud data
    #   cloud->width  = 15;
    #   cloud->height = 1;
    #   cloud->points.resize (cloud->width * cloud->height);
    #
    #   // Generate the data
    #   for (size_t i = 0; i < cloud->points.size (); ++i)
    #   {
    #     cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    #     cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    #     cloud->points[i].z = 1.0;
    #   }
    #
    #   // Set a few outliers
    #   cloud->points[0].z = 2.0;
    #   cloud->points[3].z = -2.0;
    #   cloud->points[6].z = 4.0;
    ###
    cloud = pcl.PointCloud()

    # points = np.zeros((15, 3), dtype=np.float32)
    # RAND_MAX = 1024.0
    # for i in range(0, 15):
    #     points[i][0] = 1024 * random.random() / (RAND_MAX + 1.0)
    #     points[i][1] = 1024 * random.random() / (RAND_MAX + 1.0)
    #     points[i][2] = 1.0

    # points[0][2] = 2.0
    # points[3][2] = -2.0
    # points[6][2] = 4.0
    ###
    plydata = PlyData.read(ply_file)
    ply_x = plydata['vertex']['x']
    ply_y = plydata['vertex']['y']
    ply_z = plydata['vertex']['z']
    
    points = np.zeros((ply_x.shape[0], 3), dtype=np.float32)
    points[:, 0] = ply_x
    points[:, 1] = ply_y
    points[:, 2] = ply_z

    cloud.from_array(points)
    
    #   std::cerr << "Point cloud data: " << cloud->points.size () << " points" << std::endl;
    #   for (size_t i = 0; i < cloud->points.size (); ++i)
    #     std::cerr << "    " << cloud->points[i].x << " "
    #                         << cloud->points[i].y << " "
    #                         << cloud->points[i].z << std::endl;
    #
    print('Point cloud data: ' + str(cloud.size) + ' points')
    # for i in range(0, cloud.size):
    #     print('x: ' + str(cloud[i][0]) + ', y : ' +
    #           str(cloud[i][1]) + ', z : ' + str(cloud[i][2]))

    #   pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    #   pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    #   // Create the segmentation object
    #   pcl::SACSegmentation<pcl::PointXYZ> seg;
    #   // Optional
    #   seg.setOptimizeCoefficients (true);
    #   // Mandatory
    #   seg.setModelType (pcl::SACMODEL_PLANE);
    #   seg.setMethodType (pcl::SAC_RANSAC);
    #   seg.setDistanceThreshold (0.01);
    #
    #   seg.setInputCloud (cloud);
    #   seg.segment (*inliers, *coefficients);
    ###
    # http://www.pcl-users.org/pcl-SACMODEL-CYLINDER-is-not-working-td4037530.html
    # NG?
    # seg = cloud.make_segmenter()
    # seg.set_optimize_coefficients(True)
    # seg.set_model_type(pcl.SACMODEL_NORMAL_PLANE)
    # seg.set_method_type(pcl.SAC_RANSAC)
    # seg.set_distance_threshold(0.01)
    # indices, coefficients = seg.segment()
    seg = cloud.make_segmenter_normals(ksearch=50)
    seg.set_optimize_coefficients(True)
    seg.set_model_type(pcl.SACMODEL_NORMAL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    seg.set_distance_threshold(0.01)
    seg.set_normal_distance_weight(0.005)
    seg.set_max_iterations(500)
    indices, coefficients = seg.segment()

    #   if (inliers->indices.size () == 0)
    #   {
    #     PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    #     return (-1);
    #   }
    #   std::cerr << "Model coefficients: " << coefficients->values[0] << " "
    #                                       << coefficients->values[1] << " "
    #                                       << coefficients->values[2] << " "
    #                                       << coefficients->values[3] << std::endl;
    ###
    if len(indices) == 0:
        print('Could not estimate a planar model for the given dataset.')
        exit(0)

    print('Model coefficients: ' + str(coefficients[0]) + ' ' + str(
        coefficients[1]) + ' ' + str(coefficients[2]) + ' ' + str(coefficients[3]))

    #   std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
    #   for (size_t i = 0; i < inliers->indices.size (); ++i)
    #     std::cerr << inliers->indices[i] << "    " << cloud->points[inliers->indices[i]].x << " "
    #                                                << cloud->points[inliers->indices[i]].y << " "
    #                                                << cloud->points[inliers->indices[i]].z << std::endl;
    ###
    print('Model inliers: ' + str(len(indices)))
    # for i in range(0, len(indices)):
    #     print(str(indices[i]) + ', x: ' + str(cloud[indices[i]][0]) + ', y : ' +
    #           str(cloud[indices[i]][1]) + ', z : ' + str(cloud[indices[i]][2]))

    # Delete the inliers
    plane_remove = np.delete(points, indices, axis=0)
    
    # Save to ply file
    num_verts = plane_remove.shape[0]
    verts_tuple = np.zeros((num_verts, ), dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4')])
    for i in xrange(0, num_verts):
        verts_tuple[i] = tuple(plane_remove[i, :])
    
    el_verts = PlyElement.describe(verts_tuple, 'vertex')

    ply_new_data = PlyData([el_verts])
    ply_write = ply_new_data.write(ply_output_file)



if __name__ == "__main__":
    # import cProfile
    # cProfile.run('main()', sort='time')
    ply_file = 'tsdf-fusion/model/test_1124_aruco.ply'
    ply_output_file = 'tsdf-fusion/model/test_1124_aruco_remove.ply'
    main(ply_file, ply_output_file)