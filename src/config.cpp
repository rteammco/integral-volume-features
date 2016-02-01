#include "config.h"

#include <fstream>
#include <string>


namespace iv_descriptor {

static const std::string DEFAULT_POINT_CLOUD_FILE_NAME = "../data/example.pcd";
static const float DEFAULT_RARE_KEYPOINT_FRACTION = 0.01;
static const bool DEFAULT_DISPLAY_GRID = false;

Config::Config(const std::string &config_file_name) {
  // TODO: allow the order and number of items in the config file to be
  // arbitrary. This requires better file processing.
  std::ifstream infile(config_file_name);
  if (infile) {  // Read params if the file is available.
    infile >> point_cloud_file_name_;
    infile >> rare_keypoint_fraction_;
    infile >> display_grid_;
    if (infile >> voxel_grid_file_name_) {
      has_voxel_grid_file_name_ = true;
      infile >> read_voxel_grid_;
    } else {
      has_voxel_grid_file_name_ = false;
    }
  } else {  // Otherwise set default params and write them to the file.
    point_cloud_file_name_ = DEFAULT_POINT_CLOUD_FILE_NAME;
    rare_keypoint_fraction_ = DEFAULT_RARE_KEYPOINT_FRACTION;
    display_grid_ = DEFAULT_DISPLAY_GRID;
    // Write the values to the file.
    std::ofstream outfile(config_file_name);
    outfile << point_cloud_file_name_ << "\n";
    outfile << rare_keypoint_fraction_ << "\n";
    outfile << display_grid_;
    outfile.close();
  }
}

std::string Config::GetPointCloudFileName() const {
  return point_cloud_file_name_;
}

float Config::GetRareKeypointFraction() const {
  return rare_keypoint_fraction_;
}

bool Config::HasVoxelGridFileName() const {
  return has_voxel_grid_file_name_;
}

bool Config::ReadVoxelGridFromFile() const {
  return read_voxel_grid_;
}

std::string Config::GetVoxelGridFileName() const {
  if (!has_voxel_grid_file_name_) {
    return "";
  }
  return voxel_grid_file_name_;
}

bool Config::IsDisplayGridEnabled() const {
  return display_grid_;
}

};  // namespace iv_descriptor
