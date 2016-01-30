#include "voxel_grid.h"

#include <cmath>
#include <pcl/point_types.h>


namespace iv_descriptor {

VoxelGrid::VoxelGrid(const float cell_size, const GridBounds &bounds) 
    : cell_size_(cell_size) {
  // Take abs values just in case the bounds are invalid.
  const float length_x = abs(bounds.max_x - bounds.min_x);
  const float length_y = abs(bounds.max_y - bounds.min_y);
  const float length_z = abs(bounds.max_z - bounds.min_z);
  max_x_index_ = int(ceil(length_x / cell_size_)) + 1;
  max_y_index_ = int(ceil(length_y / cell_size_)) + 1;
  max_z_index_ = int(ceil(length_z / cell_size_)) + 1;
  min_x_ = bounds.min_x - cell_size_;
  min_y_ = bounds.min_y - cell_size_;
  min_z_ = bounds.min_z - cell_size_;
}

VoxelGrid::VoxelGrid(
    const float cell_size, const pcl::PointCloud<pcl::PointXYZ> &cloud)
    : cell_size_(cell_size) {
  // TODO
}

float VoxelGrid::ConvolveAtPoint(const VoxelGrid &filter,
    const float x, const float y, const float z) const {
  // TODO
  return 0;
}

float VoxelGrid::ConvolveAtCell(
    const VoxelGrid &filter, const int x, const int y, const int z) const {
  // TODO
  return 0;
}

void VoxelGrid::AddToViewer() const {
  // TODO
}

};
