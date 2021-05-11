// ---------------------------------------------------------
// Author: Andy Zeng, Princeton University, 2016
// -----
// Modifier: Hongtao Wu, Johns Hopkins University, 2019
// Modification: 
//    Change the base frame from the camera fram of the first frame to the world frame
//    Change from GPU to CPU
// Usage: ./tsdf_fusion_cpu cam_K_file data_dir num_of_frame v_size v_x_dim v_y_dim v_z_dim v_x_origin v_y_origin v_z_origin
// ---------------------------------------------------------

#include <iostream>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include "utils.hpp"

// CUDA kernel function to integrate a TSDF voxel volume given depth images
void Integrate(float * cam_K, float * cam2base, float * depth_im,
               int im_height, int im_width, int voxel_grid_dim_x, int voxel_grid_dim_y, int voxel_grid_dim_z,
               float voxel_grid_origin_x, float voxel_grid_origin_y, float voxel_grid_origin_z, float voxel_size, float trunc_margin,
               float * voxel_grid_TSDF, float * voxel_grid_weight
){
  for (int pt_grid_x=0; pt_grid_x < voxel_grid_dim_x; ++pt_grid_x){
    for (int pt_grid_y=0; pt_grid_y < voxel_grid_dim_y; ++pt_grid_y){
        for (int pt_grid_z=0; pt_grid_z < voxel_grid_dim_z; ++pt_grid_z){
          // Convert voxel center from grid coordinates to base frame camera coordinates
          float pt_base_x = voxel_grid_origin_x + pt_grid_x * voxel_size;
          float pt_base_y = voxel_grid_origin_y + pt_grid_y * voxel_size;
          float pt_base_z = voxel_grid_origin_z + pt_grid_z * voxel_size;

          // Convert from base frame camera coordinates to current frame camera coordinates
          float tmp_pt[3] = {0};
          tmp_pt[0] = pt_base_x - cam2base[0 * 4 + 3];
          tmp_pt[1] = pt_base_y - cam2base[1 * 4 + 3];
          tmp_pt[2] = pt_base_z - cam2base[2 * 4 + 3];
          float pt_cam_x = cam2base[0 * 4 + 0] * tmp_pt[0] + cam2base[1 * 4 + 0] * tmp_pt[1] + cam2base[2 * 4 + 0] * tmp_pt[2];
          float pt_cam_y = cam2base[0 * 4 + 1] * tmp_pt[0] + cam2base[1 * 4 + 1] * tmp_pt[1] + cam2base[2 * 4 + 1] * tmp_pt[2];
          float pt_cam_z = cam2base[0 * 4 + 2] * tmp_pt[0] + cam2base[1 * 4 + 2] * tmp_pt[1] + cam2base[2 * 4 + 2] * tmp_pt[2];

          if (pt_cam_z <= 0)
            continue;

          int pt_pix_x = roundf(cam_K[0 * 3 + 0] * (pt_cam_x / pt_cam_z) + cam_K[0 * 3 + 2]);
          int pt_pix_y = roundf(cam_K[1 * 3 + 1] * (pt_cam_y / pt_cam_z) + cam_K[1 * 3 + 2]);
          if (pt_pix_x < 0 || pt_pix_x >= im_width || pt_pix_y < 0 || pt_pix_y >= im_height)
            continue;

          float depth_val = depth_im[pt_pix_y * im_width + pt_pix_x];

          if (depth_val <= 0.0001 || depth_val > 6)
            continue;

          float diff = depth_val - pt_cam_z;

          if (diff <= -trunc_margin)
            continue;

          // Integrate
          int volume_idx = pt_grid_z * voxel_grid_dim_y * voxel_grid_dim_x + pt_grid_y * voxel_grid_dim_x + pt_grid_x;
          float dist = fmin(1.0f, diff / trunc_margin);
          float weight_old = voxel_grid_weight[volume_idx];
          float weight_new = weight_old + 1.0f;
          voxel_grid_weight[volume_idx] = weight_new;
          voxel_grid_TSDF[volume_idx] = (voxel_grid_TSDF[volume_idx] * weight_old + dist) / weight_new;
      }
    }
  }
}

// Loads a binary file with depth data and generates a TSDF voxel volume (5m x 5m x 5m at 1cm resolution)
// Volume is aligned with respect to the camera coordinates of the first frame (a.k.a. base frame)
int main(int argc, char * argv[]) {

  // Location of camera intrinsic file
  std::string cam_K_file;

  // Location of folder containing RGB-D frames and camera pose files
  std::string data_path;

  cam_K_file = argv[1];
  data_path = argv[2];

  int base_frame_idx = 150;
  int first_frame_idx = 150;
  float num_frames = 24;

  float cam_K[3 * 3];
  float cam2world[4 * 4];
  int im_width = 640;
  int im_height = 480;
  float depth_im[im_height * im_width];

  // Voxel grid parameters
  float voxel_grid_origin_x = -0.5f; // Location of voxel grid origin in base frame camera coordinates
  float voxel_grid_origin_y = -0.5f;
  float voxel_grid_origin_z = -0.1f;
  float voxel_size = 0.006f;
  int voxel_grid_dim_x = 100;
  int voxel_grid_dim_y = 100;
  int voxel_grid_dim_z = 50;

  // Manual parameters
  if (argc > 3) {
    int counter = 3;
    std::cout << "Parsing input parameter...\n";

    num_frames = std::atof(argv[counter]);
    counter++;

    voxel_size = atof(argv[counter]);
    counter++;

    voxel_grid_dim_x = std::atoi(argv[counter]);
    counter++;

    voxel_grid_dim_y = std::atoi(argv[counter]);
    counter++;

    voxel_grid_dim_z = std::atoi(argv[counter]);
    counter++;

    voxel_grid_origin_x = std::atof(argv[counter]);
    counter++;

    voxel_grid_origin_y = std::atof(argv[counter]);
    counter++;

    voxel_grid_origin_z = std::atof(argv[counter]);
    counter++;

    std::cout << "Finish parsing input parameter!\n";
  }

  // Truncated Margin
  // To control how the truncate margin of TSDF
  float trunc_margin = voxel_size * 5;

  // Read camera intrinsics
  std::cout << "cam_K_file: " << cam_K_file << std::endl;
  std::vector<float> cam_K_vec = LoadMatrixFromFile(cam_K_file, 3, 3);
  std::copy(cam_K_vec.begin(), cam_K_vec.end(), cam_K);

  std::cout << "Intrinsic: " << cam_K_vec[0] << ", " << cam_K_vec[1] << ", " << cam_K_vec[2] << std::endl;

  // Initialize voxel grid
  float * voxel_grid_TSDF = new float[voxel_grid_dim_x * voxel_grid_dim_y * voxel_grid_dim_z];
  float * voxel_grid_weight = new float[voxel_grid_dim_x * voxel_grid_dim_y * voxel_grid_dim_z];
  for (int i = 0; i < voxel_grid_dim_x * voxel_grid_dim_y * voxel_grid_dim_z; ++i)
    voxel_grid_TSDF[i] = 1.0f;

  // Loop through each depth frame and integrate TSDF voxel grid
  for (int frame_idx = first_frame_idx; frame_idx < first_frame_idx + (int)num_frames; ++frame_idx) {
    
    std::ostringstream curr_frame_prefix;
    curr_frame_prefix << std::setw(6) << std::setfill('0') << frame_idx;

    // Read current frame depth
    std::string depth_im_file = data_path + "/frame-" + curr_frame_prefix.str() + ".depth.png";
    std::cout << "depth: " << depth_im_file << std::endl;
    ReadDepth(depth_im_file, im_height, im_width, depth_im);

    // Read base frame camera pose
    std::string cam2world_file = data_path + "/frame-" + curr_frame_prefix.str() + ".pose.txt";
    std::cout << "pose: " << cam2world_file << std::endl;
    std::vector<float> cam2world_vec = LoadMatrixFromFile(cam2world_file, 4, 4);
    std::copy(cam2world_vec.begin(), cam2world_vec.end(), cam2world);

    std::cout << "Fusing: " << frame_idx << std::endl;

    Integrate(cam_K, cam2world, depth_im,
              im_height, im_width, voxel_grid_dim_x, voxel_grid_dim_y, voxel_grid_dim_z,
              voxel_grid_origin_x, voxel_grid_origin_y, voxel_grid_origin_z, voxel_size, trunc_margin,
              voxel_grid_TSDF, voxel_grid_weight);
  }
  // Compute surface points from TSDF voxel grid and save to point cloud .ply file
  std::cout << "Saving surface point cloud (tsdf.ply)..." << std::endl;
  SaveVoxelGrid2SurfacePointCloud("tsdf.ply", voxel_grid_dim_x, voxel_grid_dim_y, voxel_grid_dim_z, 
                                  voxel_size, voxel_grid_origin_x, voxel_grid_origin_y, voxel_grid_origin_z,
                                  voxel_grid_TSDF, voxel_grid_weight, 0.2f, 0.0f);

  // Save TSDF voxel grid and its parameters to disk as binary file (float array)
  std::cout << "Saving TSDF voxel grid values to disk (tsdf.bin)..." << std::endl;
  std::string voxel_grid_saveto_path = "tsdf.bin";
  std::ofstream outFile(voxel_grid_saveto_path.c_str(), std::ios::binary | std::ios::out);
  float voxel_grid_dim_xf = (float) voxel_grid_dim_x;
  float voxel_grid_dim_yf = (float) voxel_grid_dim_y;
  float voxel_grid_dim_zf = (float) voxel_grid_dim_z;
  outFile.write((char*)&voxel_grid_dim_xf, sizeof(float));
  outFile.write((char*)&voxel_grid_dim_yf, sizeof(float));
  outFile.write((char*)&voxel_grid_dim_zf, sizeof(float));
  outFile.write((char*)&voxel_grid_origin_x, sizeof(float));
  outFile.write((char*)&voxel_grid_origin_y, sizeof(float));
  outFile.write((char*)&voxel_grid_origin_z, sizeof(float));
  outFile.write((char*)&voxel_size, sizeof(float));
  outFile.write((char*)&trunc_margin, sizeof(float));
  for (int i = 0; i < voxel_grid_dim_x * voxel_grid_dim_y * voxel_grid_dim_z; ++i) {
    outFile.write((char*)&voxel_grid_TSDF[i], sizeof(float));
  }
  outFile.close();

  return 0;
}