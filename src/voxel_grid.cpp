#include "voxel_grid.h"

#include <cmath>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

using pcl::PointCloud;
using pcl::PointXYZ;


namespace iv_descriptor {

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

// Point cloud voxelization constructor.
VoxelGrid::VoxelGrid(
    const float cell_size, const PointCloud<PointXYZ> &cloud)
    : cell_size_(cell_size) {
  // Set up the voxel grid dimensions.
  PointXYZ min_bounds;
  PointXYZ max_bounds;
  pcl::getMinMax3D<PointXYZ>(cloud, min_bounds, max_bounds);
  const float length_x = max_bounds.x - min_bounds.x;
  const float length_y = max_bounds.y - min_bounds.y;
  const float length_z = max_bounds.z - min_bounds.z;
  num_cells_x_ = int(ceil(length_x / cell_size_)) + 2;
  num_cells_y_ = int(ceil(length_y / cell_size_)) + 2;
  num_cells_z_ = int(ceil(length_z / cell_size_)) + 2;
  min_x_ = min_bounds.x - cell_size_;
  min_y_ = min_bounds.y - cell_size_;
  min_z_ = min_bounds.z - cell_size_;
  // Compute the border voxels.
  for (const PointXYZ &point : cloud) {
    const Index3d indices = GetGridIndex(point);
    grid_map_[indices.x][indices.y][indices.z] = 1;
  }
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

// Private method.
const Index3d VoxelGrid::GetGridIndex(const pcl::PointXYZ &point) const {
  Index3d indices;
  indices.x = int((point.x - min_x_) / cell_size_);
  indices.y = int((point.y - min_y_) / cell_size_);
  indices.z = int((point.z - min_z_) / cell_size_);
  return indices;
}

};  // namespace iv_descriptor
