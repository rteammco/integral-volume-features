// The Config class handles reading a configruation file and storing parameters
// for the algorithm. It will also write a default file if one is not provided.

#ifndef CONFIG_H
#define CONFIG_H

#include <string>


namespace iv_descriptor {

class Config {
public:
  // Attempts to read the given config file, or writes the default values to
  // it if it doesn't exist.
  Config(const std::string &config_file_name);

  // Returns the file name of the point cloud data model.
  std::string GetPointCloudFileName() const;

  // Returns the fraction of rare keypoints to use in the keypoint
  // identification step.
  float GetRareKeypointFraction() const;

  // Returns true if the option to display the voxel grid is enabled.
  bool IsDisplayGridEnabled() const;

private:
  // The name of the point cloud data file to be used.
  std::string point_cloud_file_name_;

  // The fraction of rare keypoints to select. Should be between 0 and 1.
  float rare_keypoint_fraction_;

  // Option to show the grid or not for the point cloud. Should be false if
  // using weaker hardware or very big point clouds.
  bool display_grid_;

};  // class Config

};  // namespace iv_descriptor


#endif  // CONFIG_H
