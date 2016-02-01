#include "config.h"

#include <fstream>
#include <string>


namespace iv_descriptor {

Config::Config(const std::string &config_file_name) {
  std::ifstream infile(config_file_name);
  if (infile) {  // Read params if the file is available.
    infile >> point_cloud_file_name_;
  } else {  // Otherwise set default params and write them to the file.
    point_cloud_file_name_ = DefaultPointCloudFileName();
    std::ofstream outfile(config_file_name);
    outfile << point_cloud_file_name_;
    outfile.close();
  }
}

std::string Config::GetPointCloudFileName() const {
  return point_cloud_file_name_;
}

};  // namespace iv_descriptor
