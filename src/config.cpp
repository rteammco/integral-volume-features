#include "config.h"

#include <fstream>
#include <string>


namespace iv_descriptor {

static const std::string DEFAULT_POINT_CLOUD_FILE_NAME = "../data/example.pcd";

Config::Config(const std::string &config_file_name) {
  // TODO: allow the order and number of items in the config file to be
  // arbitrary. This requires better file processing.
  std::ifstream infile(config_file_name);
  if (infile) {  // Read params if the file is available.
    infile >> point_cloud_file_name_;
    infile >> display_grid_;
  } else {  // Otherwise set default params and write them to the file.
    point_cloud_file_name_ = DEFAULT_POINT_CLOUD_FILE_NAME;
    display_grid_ = false;
    // Write the values to the file.
    std::ofstream outfile(config_file_name);
    outfile << point_cloud_file_name_ << "\n";
    outfile << display_grid_;
    outfile.close();
  }
}

std::string Config::GetPointCloudFileName() const {
  return point_cloud_file_name_;
}

bool Config::IsDisplayGridEnabled() const {
  return display_grid_;
}

};  // namespace iv_descriptor
