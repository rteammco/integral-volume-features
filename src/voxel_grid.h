// This file... TODO comments.

#ifndef VOXEL_GRID_H
#define VOXEL_GRID_H

#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <string>
#include <unordered_map>


namespace iv_descriptor {

// A simple struct containing a 3D index into the voxel grid.
struct Index3d {
  int x;
  int y;
  int z;
};

// A 3D grid map: GridMap[x][y][z] = val. Defines a sparse 3D grid.
using GridZ = std::unordered_map<int, float>;
using GridY = std::unordered_map<int, GridZ>;
using GridMap = std::unordered_map<int, GridY>;

// Defines a voxel grid.
class VoxelGrid {
  public:
    // Computes an appropriate voxel cell size using the point cloud resolution
    // method.
    static float EstimatePointCloudResolution(
        const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud);

    // Builds the voxel grid using the bounds given by the point cloud. Each
    // voxel will be a cube with edge length set to cell_size.
    VoxelGrid(const float cell_size,
              const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud);

    // Returns a cubic VoxelGrid that contains a voxelized ball (sphere) of
    // the given radius. Voxels that are inside the sphere have a value of 1,
    // and all other voxels get a value of 0.
    VoxelGrid(const float cell_size, const int radius);

    // Computes internal voxels using the watertight voxel method. Voxels that
    // are found to be internal will receive a value of 1. All other voxels
    // will have a value of 0.
    void ComputeWatertightVoxelRepresentation();

    // A const getter that returns the value at cell position (x, y, z) in the
    // grid. If the value doesn't exist (i.e. the given coordinates are out of
    // bounds, 0 will be returned instead. The cells are 0-indexed.
    float GetValueAtCell(const int x, const int y, const int z) const;

    // Performs a single convolution operation of the given filter at the given
    // (x, y, z) location in 3D space. This location is translated into cell
    // coordinates and then the convolution is computed with ConvolveAtCell().
    //
    // The given filter must have the same voxel cell size as this voxel grid.
    float ConvolveAtPoint(const VoxelGrid &filter,
                          const pcl::PointXYZ &point) const;

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
    void AddToViewer(pcl::visualization::PCLVisualizer *viewer) const;

    // Returns the size string in the form of "W x H x D" where W is width,
    // H is height, and D is depth (x, y, z dimensions, respectively).
    std::string GetSizeString() const;

  private:
    // Grid size variables.
    float cell_size_;
    float min_x_;
    float min_y_;
    float min_z_;
    int num_cells_x_;
    int num_cells_y_;
    int num_cells_z_;

    // The GridMap contains all of the voxel volumes. Access as follows:
    //   grid_map_[x][y][z] = val;
    // Writing is possible to any arbitrary coordinate. Reading from an
    // undefined coordinate will automatically return a default value of 0.
    GridMap grid_map_;

    // Computes the 3D grid index for a given point.
    const Index3d GetGridIndex(const pcl::PointXYZ &point) const;

};  // class VoxelGrid

};  // namespace iv_descriptor


#endif  // VOXEL_GRID_H
