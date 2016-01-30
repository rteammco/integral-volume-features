#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <string>

#include "voxel_grid.h"

using pcl::PointCloud;
using pcl::PointXYZ;
using pcl::visualization::PCL_VISUALIZER_POINT_SIZE;
using pcl::visualization::PCLVisualizer;
using pcl::visualization::PointCloudColorHandlerCustom;


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
  PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
  // Load the file.
  //pcl::PLYReader reader;
  pcl::PCDReader reader;
  if (reader.read<PointXYZ>(filename, *cloud) == -1) {
    PCL_ERROR("Couldn't read file!\n");
    return -1;
  }
  std::cout << "Loaded " << cloud->size() << " points." << std::endl;
  // Load the PCL 3D visualization window.
  PCLVisualizer viewer("3D Viewer");
  viewer.setBackgroundColor(0, 0, 0);
  PointCloudColorHandlerCustom<PointXYZ> single_color(cloud, 0, 255, 0);
  viewer.addPointCloud<PointXYZ>(cloud, single_color, "Point Cloud");
  viewer.setPointCloudRenderingProperties(
      PCL_VISUALIZER_POINT_SIZE, 1, "Point Cloud");
  viewer.addCoordinateSystem(1.0);
  viewer.initCameraParameters();
  viewer.spin();
  return 0;
}
