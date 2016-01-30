// This file... TODO comments.

#ifndef VOXEL_GRID_H
#define VOXEL_GRID_H

#include <pcl/point_types.h>
#include <unordered_map>


namespace iv_descriptor {

// A 3D grid map: GridMap[x][y][z] = val. Defines a sparse 3D grid.
using GridZ = std::unordered_map<int, int>;
using GridY = std::unordered_map<int, GridZ>;
using GridMap = std::unordered_map<int, GridY>;

// Defines grid bounds.
struct GridBounds {
  float min_x = 0;
  float max_x = 0;
  float min_y = 0;
  float max_y = 0;
  float min_z = 0;
  float max_z = 0;
};

// Defines a voxel grid.
class VoxelGrid {
  public:
    // Constructs a voxel grid of square voxels of the given edge length, and
    // creates the grid to fit the given bounds. At least one extra cell will be
    // padded between the minimum and maximum bound for each dimension.
    VoxelGrid(const float cell_size, const GridBounds &bounds);
    VoxelGrid(const float cell_size,
              const pcl::PointCloud<pcl::PointXYZ> &cloud);

    // Performs a single convolution operation of the given filter at the given
    // (x, y, z) location in 3D space. This location is translated into cell
    // coordinates and then the convolution is computed with ConvolveAtCell().
    //
    // The given filter must have the same voxel cell size as this voxel grid.
    float ConvolveAtPoint(const VoxelGrid &filter,
                          const float x, const float y, const float z) const;

    // Performs a single convolution operation of the given filter at the given
    // (x, y, z) grid cell location. This is effectively an intersection dot
    // product between this grid and the given filter grid centered at
    // (x, y, z). Returns the sum of the dot product of the intersection.
    //
    // The volume will be computed even if the cell indices are out of bounds by
    // only considering cells which are already defined.
    //
    // The given filter must have the same voxel cell size as this voxel grid.
    float ConvolveAtCell(
        const VoxelGrid &filter, const int x, const int y, const int z) const;

    // Adds the voxel lines to be displayed in the 3D viewer for visualization.
    void AddToViewer() const;

  private:
    const float cell_size_;
    float min_x_;
    float min_y_;
    float min_z_;
    int max_x_index_;
    int max_y_index_;
    int max_z_index_;

    // The GridMap contains all of the voxel volumes. Access as follows:
    //   grid_map_[x][y][z] = val;
    // Writing is possible to any arbitrary coordinate. Reading from an
    // undefined coordinate will automatically return a default value of 0.
    GridMap grid_map_;

};  // class VoxelGrid

};  // namespace iv_descriptor


#endif  // VOXEL_GRID_H
