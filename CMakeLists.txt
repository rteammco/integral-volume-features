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
# Find the GTest library files.
find_library(gtest_lib gtest libgtest.a libgtest)
find_library(gtest_main_lib gtest_main libgtest_main.a libgtest_main)
# Find the GMock library files.
find_library(gmock_lib gmock libgmock.a libgmock)
find_library(gmock_main_lib gmock_main libgmock_main.a libgmock_main)
# Link the located gtest and gmock libraries to the test binary.
target_link_libraries(${proj_test_bin} ${gtest_lib} ${gtest_main_lib})
target_link_libraries(${proj_test_bin} ${gmock_lib} ${gmock_main_lib})
# Link threads library to the test binary.
target_link_libraries(${proj_test_bin} ${CMAKE_THREAD_LIBS_INIT})
# Adds test to Makefile (run "make test"):
add_test(test1 ${proj_test_bin})
