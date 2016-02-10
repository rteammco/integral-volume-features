cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(IVFEATURES)


# Set the compiler to C++11:
# http://stackoverflow.com/questions/10851247/how-to-activate-c-11-in-cmake
set(CMAKE_CXX_FLAGS "-std=c++11")


# Add PCL to the package:
find_package(PCL 1.3 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


# Add gtest (the Google test library) to the package:
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})


# Build for the main executable:
file(GLOB ivfeatures_src "src/*.h" "src/*.cpp")
add_executable(ivfeatures ${ivfeatures_src})
target_link_libraries(ivfeatures ${PCL_LIBRARIES})
