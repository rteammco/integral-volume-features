#include "./voxel_grid.h"

#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include <stdlib.h>

#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include <iostream>  // TODO(richard): remove

using pcl::PointCloud;
using pcl::PointXYZ;


namespace iv_descriptor {

namespace {

// Random sampling method: returns k integers between 0 and n-1 sampled
// randomly. This is based on the Fisher–Yates shuffle algorithm.
std::vector<int> GetRandomSamples(const int n, const int k) {
  std::vector<int> samples;
  samples.reserve(k);
  std::unordered_map<int, int> assigned;
  for (int i = 0; i < k; ++i) {
    const int r = rand() % (n - i);
    if (assigned.find(r) == assigned.end()) {
      samples.push_back(r);
    } else {
      samples.push_back(assigned[r]);
    }
    assigned[r] = (n - i) - 1;
  }
  return samples;
}

// Adds lines for the given primary axis d0. d1 and d2 should be indices of the
// other two axes. For example, to draw lines in the x-axis, set d0 to 0, and d1
// and d2 to 1 and 2, respectively. To draw lines across the y-axis, set d0 to
// 1, and d1 and d2 to 0 and 2, respectively, and so on. The order of d1 and d2
// does not matter. min_d# is the minimum value in the #-axis. Similarly, num_d#
// is the number of lines to be drawn in the #-axis.
void DrawLinesForAxis(pcl::visualization::PCLVisualizer *viewer,
    const float cell_size, const int d0, const int d1, const int d2,
    const float min_d0, const float min_d1, const float min_d2,
    const int num_d0, const int num_d1, const int num_d2) {
  std::vector<float> line_start = {0, 0, 0};
  std::vector<float> line_end = {0, 0, 0};
  line_start[d0] = min_d0;
  line_end[d0] = min_d0 + num_d0 * cell_size;
  std::ostringstream id_sstream;
  for (int i = 0; i <= num_d1; ++i) {
    const float pos_i = min_d1 + i * cell_size;
    line_start[d1] = pos_i;
    line_end[d1] = pos_i;
    for (int j = 0; j <= num_d2; ++j) {
      const float pos_j = min_d2 + j * cell_size;
      line_start[d2] = pos_j;
      line_end[d2] = pos_j;
      PointXYZ p0(line_start[0], line_start[1], line_start[2]);
      PointXYZ p1(line_end[0], line_end[1], line_end[2]);
      id_sstream.str("");
      id_sstream << "line_" << d0 << "_" << i << "_" << j;
      viewer->addLine<PointXYZ>(p0, p1, id_sstream.str());
    }
  }
}

}  // namespace

// Static method.
float VoxelGrid::EstimatePointCloudResolution(
    const PointCloud<PointXYZ>::ConstPtr &cloud) {
  // TODO(richard): These should be parameters (7 / 50).
  const int num_nearest_neighbors = 7;
  const int max_sample_count = 50;
  float total_distances = 0;
  int total_count = 0;
  // Randomly choose 50 points in the point cloud (or all if there are less
  // than 50 available).
  const int num_samples =
      (cloud->size() >= max_sample_count) ? max_sample_count : cloud->size();
  std::vector<int> sample_indices = GetRandomSamples(
      cloud->size(), num_samples);
  // For each, look up its 7 nearest neighbors, and take the average distance.
  pcl::KdTreeFLANN<PointXYZ> kdtree;
  kdtree.setInputCloud(cloud);
  for (const int index : sample_indices) {
    const PointXYZ &point = cloud->points[index];
    std::vector<int> neighbor_indices(num_nearest_neighbors);
    std::vector<float> neighbor_distances_squared(num_nearest_neighbors);
    // Search for 1 extra nearest neighbor, since the point itself will come up
    // as one of the results.
    const int num_found = kdtree.nearestKSearch(
        point, num_nearest_neighbors + 1,
        neighbor_indices, neighbor_distances_squared);
    for (int i = 0; i < num_found; ++i) {
      if (neighbor_indices[i] != index) {
        total_distances += sqrt(neighbor_distances_squared[i]);
        total_count++;
      }
    }
  }
  if (total_count == 0) {
    return 0;
  }
  return total_distances / total_count;
}

// Default constructor.
VoxelGrid::VoxelGrid() {
  cell_size_ = 0;
  min_x_ = 0;
  min_y_ = 0;
  min_z_ = 0;
  num_cells_x_ = 0;
  num_cells_y_ = 0;
  num_cells_z_ = 0;
}

// Ball grid constructor.
VoxelGrid::VoxelGrid(const float cell_size, const int radius)
    : cell_size_(cell_size) {
  const int diameter = 2 * radius;
  num_cells_x_ = diameter;
  num_cells_y_ = diameter;
  num_cells_z_ = diameter;
  const float min_value = -cell_size_ * radius;
  min_x_ = min_value;
  min_y_ = min_value;
  min_z_ = min_value;
  // Compute the interior of the sphere.
  const float r = radius * cell_size_;
  const float r2 = r * r;
  for (int i = 0; i < num_cells_x_; ++i) {
    const float dx = min_x_ + i * cell_size_;
    for (int j = 0; j < num_cells_y_; ++j) {
      const float dy = min_y_ + j * cell_size_;
      for (int k = 0; k < num_cells_z_; ++k) {
        const float dz = min_z_ + k * cell_size_;
        const float d2 = (dx * dx) + (dy * dy) + (dz * dz);
        if (d2 <= r2) {
          grid_map_[i][j][k] = 1;
        }
      }
    }
  }
}

// Point cloud voxelization constructor.
void VoxelGrid::BuildAroundPointCloud(
    const float cell_size, const PointCloud<PointXYZ>::ConstPtr &cloud) {
  // Set up the voxel grid dimensions.
  cell_size_ = cell_size;
  PointXYZ min_bounds;
  PointXYZ max_bounds;
  pcl::getMinMax3D<PointXYZ>(*cloud, min_bounds, max_bounds);
  const float length_x = max_bounds.x - min_bounds.x;
  const float length_y = max_bounds.y - min_bounds.y;
  const float length_z = max_bounds.z - min_bounds.z;
  num_cells_x_ = static_cast<int>(ceil(length_x / cell_size_)) + 2;
  num_cells_y_ = static_cast<int>(ceil(length_y / cell_size_)) + 2;
  num_cells_z_ = static_cast<int>(ceil(length_z / cell_size_)) + 2;
  min_x_ = min_bounds.x - cell_size_;
  min_y_ = min_bounds.y - cell_size_;
  min_z_ = min_bounds.z - cell_size_;
  // Compute the border voxels.
  for (const PointXYZ &point : cloud->points) {
    const Index3d indices = GetGridIndex(point);
    grid_map_[indices.x][indices.y][indices.z] = 1;
  }
}

// Load from file constructor.
bool VoxelGrid::LoadFromFile(const std::string &file_name) {
  std::ifstream infile(file_name);
  if (!infile) {
    return false;
  }
  // Read the necessary metadata.
  // TODO(richard): check for failures here.
  infile >> cell_size_;
  infile >> min_x_ >> min_y_ >> min_z_;
  infile >> num_cells_x_ >> num_cells_y_ >> num_cells_z_;
  // Then read all of the non-zero voxel values.
  int x, y, z;
  float val;
  while (infile >> x >> y >> z >> val) {
    grid_map_[x][y][z] = val;
  }
  infile.close();
  return true;
}

void VoxelGrid::ComputeWatertightVoxelRepresentation() {
  // TODO(richard): this is repeating code. Make it a function somehow.
  // Compute in the x-direction.
  for (int y = 0; y < num_cells_y_; ++y) {
    for (int z = 0; z < num_cells_z_; ++z) {
      bool inside = false;
      int start_index = 0;
      for (int x = 0; x < num_cells_x_; ++x) {
        if (GetValueAtCell(x, y, z) == 1) {
          // If encountered a second surface cell, subtract 1 between the two
          // surfaces.
          if (inside) {
            for (int a = start_index; a < x; ++a) {
              grid_map_[a][y][z]--;
            }
          } else {  // Otherwise, set the start index.
            start_index = x + 1;
          }
          inside = !inside;
        }
      }
    }
  }
  // Compute in the y-direction.
  for (int x = 0; x < num_cells_x_; ++x) {
    for (int z = 0; z < num_cells_z_; ++z) {
      bool inside = false;
      int start_index = 0;
      for (int y = 0; y < num_cells_y_; ++y) {
        if (GetValueAtCell(x, y, z) == 1) {
          if (inside) {
            for (int a = start_index; a < y; ++a) {
              grid_map_[x][a][z]--;
            }
          } else {
            start_index = y + 1;
          }
          inside = !inside;
        }
      }
    }
  }
  // Compute in the z-direction.
  for (int x = 0; x < num_cells_x_; ++x) {
    for (int y = 0; y < num_cells_y_; ++y) {
      bool inside = false;
      int start_index = 0;
      for (int z = 0; z < num_cells_z_; ++z) {
        if (GetValueAtCell(x, y, z) == 1) {
          if (inside) {
            for (int a = start_index; a < z; ++a) {
              grid_map_[x][y][a]--;
            }
          } else {
            start_index = z + 1;
          }
          inside = !inside;
        }
      }
    }
  }
  // Switch all cells with a value of -2 or lower to 1, and 0 otherwise.
  for (int x = 0; x < num_cells_x_; ++x) {
    for (int y = 0; y < num_cells_y_; ++y) {
      for (int z = 0; z < num_cells_z_; ++z) {
        const float cell_value = GetValueAtCell(x, y, z);
        // If the value is <= 2, it becomes an internal voxel. Otherwise, reset
        // it to 0 (only if it isn't already 0).
        if (cell_value <= -2) {
          grid_map_[x][y][z] = 1;
        } else if (cell_value != 0) {
          grid_map_[x][y][z] = 0;
        }
      }
    }
  }
  // TODO(richard): Check all cells to eliminate and tubular holes caused by
  // noise.
}

float VoxelGrid::GetValueAtCell(const int x, const int y, const int z) const {
  // Search the x coordinate first. Return 0 if it doesn't exist.
  const GridMap::const_iterator x_iter = grid_map_.find(x);
  if (x_iter == grid_map_.end()) {
    return 0;
  }
  // Now search the y coordinate.
  const GridY &grid_y = grid_map_.at(x_iter->first);
  const GridY::const_iterator y_iter = grid_y.find(y);
  if (y_iter == grid_y.end()) {
    return 0;
  }
  // Finally, search the z coordinate.
  const GridZ &grid_z = grid_y.at(y_iter->first);
  const GridZ::const_iterator z_iter = grid_z.find(z);
  if (z_iter == grid_z.end()) {
    return 0;
  }
  return grid_z.at(z_iter->first);
}

float VoxelGrid::ConvolveAtPoint(
    const VoxelGrid &filter, const PointXYZ &point) const {
  const Index3d indices = GetGridIndex(point);
  return ConvolveAtCell(filter, indices.x, indices.y, indices.z);
}

float VoxelGrid::ConvolveAtCell(
    const VoxelGrid &filter, const int x, const int y, const int z) const {
  // If cell sizes don't match, return 0.
  if (cell_size_ != filter.cell_size_) {
    return 0;
  }
  // Run the convolution centered at the given index.
  float total = 0;
  const int filter_center_x = filter.num_cells_x_ / 2;
  const int filter_center_y = filter.num_cells_y_ / 2;
  const int filter_center_z = filter.num_cells_z_ / 2;
  for (int i = 0; i < filter.num_cells_x_; ++i) {
    for (int j = 0; j < filter.num_cells_y_; ++j) {
      for (int k = 0; k < filter.num_cells_z_; ++k) {
        const float filter_val = filter.GetValueAtCell(i, j, k);
        const float grid_val = GetValueAtCell(
            x + (i - filter_center_x),
            y + (j - filter_center_y),
            z + (k - filter_center_z));
        total += filter_val * grid_val;
      }
    }
  }
  return total;
}

void VoxelGrid::AddToViewer(pcl::visualization::PCLVisualizer *viewer) const {
  // Fill in cells that have a density value.
  const float half_cell_size = cell_size_ / 2;
  const float quarter_cell_size = cell_size_ / 4;
  std::ostringstream id_sstream;
  for (int i = 0; i < num_cells_x_; ++i) {
    const float x = min_x_ + i * cell_size_ + half_cell_size;
    for (int j = 0; j < num_cells_y_; ++j) {
      const float y = min_y_ + j * cell_size_ + half_cell_size;
      for (int k = 0; k < num_cells_z_; ++k) {
        if (GetValueAtCell(i, j, k) > 0) {
          const float z = min_z_ + k * cell_size_ + half_cell_size;
          id_sstream.str("");
          id_sstream << "inner_" << i << "_" << j << "_" << k;
          viewer->addSphere(PointXYZ(x, y, z), quarter_cell_size,
                            255, 0, 0, id_sstream.str());
        }
      }
    }
  }
  // Draw lines for the x, y, and z axes, respectively.
  DrawLinesForAxis(viewer, cell_size_, 0, 1, 2, min_x_, min_y_, min_z_,
      num_cells_x_, num_cells_y_, num_cells_z_);
  DrawLinesForAxis(viewer, cell_size_, 1, 0, 2, min_y_, min_x_, min_z_,
      num_cells_y_, num_cells_x_, num_cells_z_);
  DrawLinesForAxis(viewer, cell_size_, 2, 0, 1, min_z_, min_x_, min_y_,
      num_cells_z_, num_cells_x_, num_cells_y_);
}

std::string VoxelGrid::GetSizeString() const {
  std::ostringstream size_sstream;
  size_sstream <<
      num_cells_x_ << " x " << num_cells_y_ << " x " << num_cells_z_;
  return size_sstream.str();
}

float VoxelGrid::GetVoxelSize() const {
  return cell_size_;
}

void VoxelGrid::ExportToFile(const std::string &file_name) const {
  std::ofstream outfile(file_name);
  if (!outfile) {
    // TODO(richard): error logging?
    return;
  }
  // Write the necessary metadata.
  outfile << cell_size_ << " ";
  outfile << min_x_ << " " << min_y_ << " " << min_z_ << " ";
  outfile << num_cells_x_ << " " << num_cells_y_ << " " << num_cells_z_ << "\n";
  // Then write all of the non-zero voxel values.
  for (int i = 0; i < num_cells_x_; ++i) {
    for (int j = 0; j < num_cells_y_; ++j) {
      for (int k = 0; k < num_cells_z_; ++k) {
        const float val = GetValueAtCell(i, j, k);
        if (val != 0) {
          outfile << i << " " << j << " " << k << " " << val << "\n";
        }
      }
    }
  }
  outfile.close();
}

// Private method.
const Index3d VoxelGrid::GetGridIndex(const pcl::PointXYZ &point) const {
  Index3d indices;
  indices.x = static_cast<int>((point.x - min_x_) / cell_size_);
  indices.y = static_cast<int>((point.y - min_y_) / cell_size_);
  indices.z = static_cast<int>((point.z - min_z_) / cell_size_);
  return indices;
}

}  // namespace iv_descriptor
