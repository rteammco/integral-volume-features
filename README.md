# Integral Volume Features

This is an implementation of the algorithms introduced by

<i>Gelfand, Natasha, et al. "Robust global registration." Symposium on geometry processing. Vol. 2. No. 3. 2005.</i>

and

<i>Garstka, Jens, and Gabriele Peters. "Fast and robust keypoint detection in unstructured 3-D point clouds." Informatics in Control, Automation and Robotics (ICINCO), 2015 12th International Conference on. Vol. 2. IEEE, 2015.</i>

Build Instructions
----------

<ol>
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

<ul>
  <li>GCC 4.7+ (uses C++11 features)</li>
  <li>CMake 2.8+: https://cmake.org/</li>
  <li>PCL (Point Cloud Library): http://pointclouds.org/</li>
  <li>Google Test: https://github.com/google/googletest</li>
</ul>
