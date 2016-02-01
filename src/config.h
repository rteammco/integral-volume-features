// The Config class handles reading a configruation file and storing parameters
// for the algorithm. It will also write a default file if one is not provided.

#ifndef CONFIG_H
#define CONFIG_H

#include <string>


namespace iv_descriptor {

// Returns the default point cloud file name. Use this only if a configuration
// file specifying the user's desired data is not available.
inline const std::string& DefaultPointCloudFileName() {
  static const std::string filename("../data/example.pcd");
  return filename;
}

class Config {
public:
  // Attempts to read the given config file, or writes the default values to
  // it if it doesn't exist.
  Config(const std::string &config_file_name);

  // Returns the file name of the point cloud data model.
  std::string GetPointCloudFileName() const;

private:
  // The name of the point cloud data file to be used.
  std::string point_cloud_file_name_;

};  // class Config

};  // namespace iv_descriptor


#endif  // CONFIG_H
