// The Config class handles reading a configruation file and storing parameters
// for the algorithm. It will also write a default file if one is not provided.

#ifndef SRC_CONFIG_H_
#define SRC_CONFIG_H_

#include <string>


namespace iv_descriptor {

class Config {
 public:
  // Attempts to read the given config file, or writes the default values to
  // it if it doesn't exist.
  explicit Config(const std::string &config_file_name);

  // Returns the file name of the point cloud data model.
  std::string GetPointCloudFileName() const;

  // Returns the fraction of rare keypoints to use in the keypoint
  // identification step.
  float GetRareKeypointFraction() const;

  // Returns true if the user provided a file name from which to load the voxel
  // grid instead of rebuilding it or save the grid to for exporting.
  bool HasVoxelGridFileName() const;

  // Returns true if the user wanted the voxel grid to be loaded from a file
  // and a file name was provided.
  bool ReadVoxelGridFromFile() const;

  // Returns true if the user wanted the voxel grid to be written out to
  // a file and provided the file name. This method returns the opposite of
  // ReadFromGridFile if the file name is provided.
  bool WriteVoxelGridToFile() const;

  // Returns the filename of the voxel grid file if the user specified loading
  // the voxel grid from a file. Otherwise, an empty string is returned.
  std::string GetVoxelGridFileName() const;

  // Returns true if the option to display the voxel grid is enabled.
  bool IsDisplayGridEnabled() const;

 private:
  // The name of the point cloud data file to be used.
  std::string point_cloud_file_name_;

  // The fraction of rare keypoints to select. Should be between 0 and 1.
  float rare_keypoint_fraction_;

  // True if a voxel grid file name was provided.
  bool has_voxel_grid_file_name_;

  // True if the voxel grid file (if provided) should be read from to load the
  // voxel grid. If false, the user intends to write the voxel grid to that file
  // instead.
  bool read_voxel_grid_;

  // The name of the voxel grid file (if provided).
  std::string voxel_grid_file_name_;

  // Option to show the grid or not for the point cloud. Should be false if
  // using weaker hardware or very big point clouds.
  bool display_grid_;
};  // class Config

};  // namespace iv_descriptor


#endif  // SRC_CONFIG_H_
