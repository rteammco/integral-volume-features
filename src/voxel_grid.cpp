#include "voxel_grid.h"


namespace iv_descriptor {

VoxelGrid::VoxelGrid(const float cell_size, const GridBounds &bounds) 
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
