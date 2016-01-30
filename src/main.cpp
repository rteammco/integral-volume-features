#include <iostream>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include "voxel_grid.h"


int main (int argc, char **argv) {
  iv_descriptor::VoxelGrid g;
  //const std::string filename = "../data/bun000.ply";
  const std::string filename = "../data/table_scene_lms400.pcd";
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  // Load the file.
  //pcl::PLYReader reader;
  pcl::PCDReader reader;
  if (reader.read<pcl::PointXYZ>(filename, *cloud) == -1) {
    PCL_ERROR("Couldn't read file!\n");
    return -1;
  }
  std::cout << "Loaded " << cloud->size() << " points." << std::endl;
  pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
  viewer.showCloud(cloud);
  while (!viewer.wasStopped()) {}
  return 0;
}
