// An example showing TEASER++ registration with FPFH features with the Stanford bunny model
#include <chrono>
#include <iostream>
#include <random>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <filesystem>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>

#include "tinyply.h"
#include "ply_io.h"

namespace fs = std::filesystem;

int main(int argc, char** argv) {
  if (argc < 4) {
      std::cerr << "Usage: " << argv[0] << " <input_folder> <voxel_size> <output_file.ply>" << std::endl;
      return -1;
  }

  std::string input_folder = argv[1];
  float voxel_size = std::stof(argv[2]);
  std::string output_file = argv[3];

  teaser::PLYReader reader;


  pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
  voxel_grid.setLeafSize(0.25, 0.25, 0.25); // For saving memory
  pcl::PointCloud<pcl::PointXYZ>::Ptr voxelized_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  // Iterate through all .ply files in the input folder
  teaser::PointCloud src_cloud;
  int count = 0;
  for (const auto& entry : fs::directory_iterator(input_folder)) {
    if (entry.path().extension() == ".ply") {
      auto status = reader.read(entry.path().string(), src_cloud);
      pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      
      for (int k = 0; k < src_cloud.size(); ++k) {
        tmp_cloud->emplace_back(pcl::PointXYZ(src_cloud[k].x, src_cloud[k].y, src_cloud[k].z));
      }
      voxel_grid.setInputCloud(tmp_cloud);
      voxel_grid.filter(*voxelized_cloud);
      *accumulated_cloud += *voxelized_cloud; // Accumulate point clouds
      std::cout << "Loaded and accumulated: " << entry.path() << std::endl;
    }

    ++count;

    // Without this part,
    // segmentation fault occurs at the final voxelization step
    if (count % 10 == 0) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      voxel_grid.setInputCloud(accumulated_cloud);
      voxel_grid.setLeafSize(voxel_size, voxel_size, voxel_size); // Adjust leaf size as needed
      std::cout << "On voxelization (it might take some time)..." << std::endl;
      voxelized_cloud->clear();
      voxel_grid.filter(*tmp_cloud);
      *accumulated_cloud = *tmp_cloud;
    }
  }

  if (accumulated_cloud->empty()) {
      std::cerr << "No .ply files loaded. Exiting." << std::endl;
      return -1;
  }

  // Apply voxel grid filter for downsampling
  voxel_grid.setInputCloud(accumulated_cloud);
  voxel_grid.setLeafSize(voxel_size, voxel_size, voxel_size); // Adjust leaf size as needed
  std::cout << "On voxelization (it might take some time)..." << std::endl;
  voxelized_cloud->clear();
  voxel_grid.filter(*voxelized_cloud);
  std::cout << "Complete!" << std::endl;

  // Save the voxelized point cloud to a .pcd file
  pcl::io::savePCDFileASCII(output_file, *voxelized_cloud);

  std::cout << "Voxelized point cloud saved to: " << output_file << std::endl;
  return 0;
}
