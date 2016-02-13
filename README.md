# Integral Volume Features

This is an implementation of the algorithms introduced by

<i>Gelfand, Natasha, et al. "Robust global registration." Symposium on geometry processing. Vol. 2. No. 3. 2005.</i>

and

<i>Garstka, Jens, and Gabriele Peters. "Fast and robust keypoint detection in unstructured 3-D point clouds." Informatics in Control, Automation and Robotics (ICINCO), 2015 12th International Conference on. Vol. 2. IEEE, 2015.</i>

Build Instructions
----------

<ol>
  <li>Make sure all the required tools and libraries are installed (see below).</li>
  <li><code>git clone https://github.com/teammcr192/integral-volume-features.git</code></li>
  <li><code>cd integral-volume-features</code></li>
  <li><code>mkdir build</code></li>
  <li><code>cd build</code></li>
  <li><code>cmake ..</code></li>
  <li><code>make</code></li>
  <li><code>./ivfeatures</code></li>
</ol>

Required Tools and Libraries
----------

The current CMakeLists.txt file only supports Linux (tested on Ubuntu 14.04) due to the current setup of the GTest library. Making it cross-platform is a TODO.

<ul>
  <li>GCC 4.7+ (uses C++11 features)</li>
  <li>CMake 2.8+: https://cmake.org/</li>
  <li>PCL (Point Cloud Library): http://pointclouds.org/
    <ul>
      <li>NOTE: The official binaries for Ubuntu are only supported up to version 14.04 (http://pointclouds.org/downloads/linux.html).</li>
    </ul>
  </li>
  <li>Google Test: https://github.com/google/googletest
    <ul>
      <li>This code uses both Google Test and Google Mock components. See <code>INSTALL_GTEST</code> for install instructions.</li>
    </ul>
  </li>
</ul>
