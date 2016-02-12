cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
set(proj_name IVFeatures)
set(proj_bin ivfeatures)
project(${proj_name} C CXX)


# Set the compiler to C++11:
# http://stackoverflow.com/questions/10851247/how-to-activate-c-11-in-cmake
set(CMAKE_CXX_FLAGS "-std=c++11")


# Add PCL to the package:
find_package(PCL 1.3 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


# Build for the main executable:
file(GLOB proj_src "src/*.h" "src/*.cpp")
add_executable(${proj_bin} ${proj_src})
target_link_libraries(${proj_bin} ${PCL_LIBRARIES})


# Add gtest (the Google test library) to the package:
find_package(GTest REQUIRED)
find_package(Threads REQUIRED)
set(proj_include "src/")
include_directories(${GTEST_INCLUDE_DIRS} ${proj_include})


# Set up tests.
enable_testing()
set(proj_test_bin ${proj_bin}_test)
file(GLOB proj_test_src "test/*.cpp")
add_executable(${proj_test_bin} ${proj_test_src})
add_dependencies(${proj_test_bin} googletest)
# TODO: gtest_libs_dir - how to make CMake automatically find it?
set(gtest_libs_dir "/usr/src/gtest")
target_link_libraries(${proj_test_bin}
      ${gtest_libs_dir}/libgtest.a
      ${gtest_libs_dir}/libgtest_main.a
)
target_link_libraries(${proj_test_bin} ${CMAKE_THREAD_LIBS_INIT})
# Adds test to Makefile (run "make test"):
add_test(test1 ${proj_test_bin})
