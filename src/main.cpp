#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <string>

#include "voxel_grid.h"


int main (int argc, char **argv) {
  // Process args.
  // Load data model.
  // Determine voxel grid cell size.
  // Build voxel grid.
  // Determine which voxels are interior and which are exterior.
  // Create grid ball.
  // For each point in the cloud, compute the intersection.
  // Compute standard deviation of the data.
  // Compute histogram bin width using Scott's rule.
  // Find bins with the rarest (<= 1%) of points.
  // Mark the appropriate points for selection - cluster, or range threshold.
  // Repeat for multiple radii of the sphere.
  // Select points that were features for at least 2 consecutive radii.

  // TODO:
  // Determine voxel grid size?
  // Building voxel grid.
  // Computing interior of voxel grid.
  // Ball Voxel Grid builder.

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
