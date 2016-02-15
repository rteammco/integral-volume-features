#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "./config.h"
#include "./histogram.h"
#include "./voxel_grid.h"

using iv_descriptor::Config;
using iv_descriptor::Histogram;
using iv_descriptor::VoxelGrid;
using pcl::PointCloud;
using pcl::PointXYZ;
using pcl::visualization::PCL_VISUALIZER_POINT_SIZE;
using pcl::visualization::PCLVisualizer;
using pcl::visualization::PointCloudColorHandlerCustom;


  // TODO(richard): move algorithm steps into a new class.
int main(int argc, char **argv) {
  // List of TODO items: "-!-" means finished.
  // -!- Process args (uses config file now).
  // -!- Load data model.
  // -!- Determine voxel grid cell size.
  // -!- Build voxel grid.
  // -!- Determine which voxels are interior and which are exterior.
  // -!- Create grid ball.
  // -!- For each point in the cloud, compute the intersection.
  // -!- Compute standard deviation of the data.
  // -!- Compute histogram bin width using Scott's rule.
  // -!- Find bins with the rarest (<= 1%) of points.
  // Mark the appropriate points for selection - cluster, or range threshold.
  // Repeat for multiple radii of the sphere.
  // Select points that were features for at least 2 consecutive radii.
  Config config("config.txt");

  // Load the data file into a PointCloud object and build the voxel grid.
  PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
  pcl::PCDReader reader;
  if (reader.read<PointXYZ>(config.GetPointCloudFileName(), *cloud) == -1) {
    PCL_ERROR("Couldn't read file!\n");
    return -1;
  }
  std::cout << "Loaded " << cloud->size() << " points." << std::endl;

  // Scale the point cloud so that it's centered and approximately 2 units
  // across in the X axis and center it on the origin.
  PointXYZ min_bounds, max_bounds;
  pcl::getMinMax3D<PointXYZ>(*cloud, min_bounds, max_bounds);
  // Center the point cloud on the origin.
  const float center_x = (max_bounds.x - min_bounds.x) / 2;
  const float center_y = (max_bounds.y - min_bounds.y) / 2;
  const float center_z = (max_bounds.z - min_bounds.z) / 2;
  Eigen::Affine3f translation_mat = Eigen::Affine3f::Identity();
  translation_mat.translation() << -center_x, -center_y, -center_z;
  pcl::transformPointCloud(*cloud, *cloud, translation_mat);
  // Now scale it.
  const float scale = 2.0 / (max_bounds.x - min_bounds.x);
  Eigen::Matrix4f scale_mat = Eigen::Matrix4f::Identity();
  scale_mat(0, 0) = scale;
  scale_mat(1, 1) = scale;
  scale_mat(2, 2) = scale;
  pcl::transformPointCloud(*cloud, *cloud, scale_mat);
  std::cout << "Cloud resized and centered." << std::endl;

  // Build the integral volume voxel grid around the point cloud or load it
  // from a file (depending on user's specifications).
  VoxelGrid voxel_grid;
  if (config.ReadVoxelGridFromFile()) {
    if (voxel_grid.LoadFromFile(config.GetVoxelGridFileName())) {
      std::cout << "Loaded grid from file." << std::endl;
    } else {
      std::cerr << "Failed to load file." << std::endl;
      return -1;
    }
  } else {
    const float voxel_size = VoxelGrid::EstimatePointCloudResolution(cloud);
    voxel_grid.BuildAroundPointCloud(voxel_size, cloud);
    voxel_grid.ComputeWatertightVoxelRepresentation();
    std::cout << "Watertight voxel representation computed." << std::endl;
    // If write to file is specified, save the voxel grid.
    if (config.WriteVoxelGridToFile()) {
      voxel_grid.ExportToFile(config.GetVoxelGridFileName());
      std::cout << "Exported grid to file." << std::endl;
    }
  }
  const float voxel_size = voxel_grid.GetVoxelSize();
  std::cout << "Voxel edge length: " << voxel_size << std::endl;
  std::cout << "Voxel grid size: " << voxel_grid.GetSizeString() << std::endl;

  // Create the ball and convolve with the voxel grid.
  VoxelGrid ball_grid(voxel_size, 10);
  std::vector<float> values;
  values.reserve(cloud->size());
  for (const PointXYZ &point : cloud->points) {
    values.push_back(voxel_grid.ConvolveAtPoint(ball_grid, point));
  }
  std::cout << "Convolution with ball done." << std::endl;

  // Create a histogram and find the rare output values.
  Histogram hist(values);
  std::vector<int> rare_indices =
      hist.GetRareValues(config.GetRareKeypointFraction());
  std::cout << "Found " << rare_indices.size() << " keypoints." << std::endl;

  // Make a point cloud of rare values only and cluster the rare values.
  PointCloud<PointXYZ>::Ptr rare_cloud(
      new PointCloud<PointXYZ>(*cloud, rare_indices));
  // TODO(richard): cluster here!

  // Load the PCL 3D visualization window and add the point cloud and voxel
  // grid to be displayed.
  PCLVisualizer viewer("3D Viewer");
  viewer.setBackgroundColor(0, 0, 0);
  PointCloudColorHandlerCustom<PointXYZ> single_color(cloud, 0, 255, 0);
  viewer.addPointCloud<PointXYZ>(cloud, single_color, "Point Cloud");
  viewer.setPointCloudRenderingProperties(
      PCL_VISUALIZER_POINT_SIZE, 1, "Point Cloud");
  if (config.IsDisplayGridEnabled()) {  // Draw the grid if enabled.
    voxel_grid.AddToViewer(&viewer);
  }
  // Draw the rare points (keypoints).
  const float keypoint_radius = voxel_size / 4;
  std::ostringstream id_sstream;
  int counter = 0;
  for (const PointXYZ &rare_point : rare_cloud->points) {
    id_sstream.str("");
    id_sstream << "keypoint_" << counter++;
    viewer.addSphere(rare_point, keypoint_radius, 255, 0, 0, id_sstream.str());
  }
  viewer.addCoordinateSystem(1.0);
  viewer.initCameraParameters();
  viewer.spin();
  return 0;
}
