#include <iostream>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>


int main (int argc, char **argv) {
  const std::string filename = "../data/example.pcd";
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  // Load the file.
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *cloud) == -1) {
    PCL_ERROR("Couldn't read file test_pcd.pcd \n");
    return -1;
  }
  pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
  viewer.showCloud(cloud);
  while (!viewer.wasStopped()) {}
  return 0;
}
