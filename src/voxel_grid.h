#ifndef VOXEL_GRID_H
#define VOXEL_GRID_H


namespace iv_descriptor {


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
};  // class VoxelGrid

}  // namespace iv_descriptor


#endif  // VOXEL_GRID_H
