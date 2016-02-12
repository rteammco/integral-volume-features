cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
set(project_name IVFeatures)
project(${project_name} C CXX)


# Set the compiler to C++11:
# http://stackoverflow.com/questions/10851247/how-to-activate-c-11-in-cmake
set(CMAKE_CXX_FLAGS "-std=c++11")


# Add PCL to the package:
find_package(PCL 1.3 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


# Build for the main executable:
file(GLOB ivfeatures_src "src/*.h" "src/*.cpp")
add_executable(ivfeatures ${ivfeatures_src})
target_link_libraries(ivfeatures ${PCL_LIBRARIES})


# Add gtest (the Google test library) to the package:
find_package(GTest REQUIRED)
set(ivfeatures_include "src/")
include_directories(${GTEST_INCLUDE_DIRS} ${ivfeatures_include})

# Test stuff.
find_package(Threads REQUIRED)

enable_testing()
set(project_test_name ${project_name}_test)

file(GLOB ivfeatures_test_src "test/*.cpp")
add_executable(${project_test_name} ${ivfeatures_test_src})
add_dependencies(${project_test_name} googletest)

target_link_libraries(${project_test_name}
      /usr/src/gtest/libgtest.a
      /usr/src/gtest/libgtest_main.a
#      ${GTEST_LIBS_DIR}/libgtest.a
#      ${GTEST_LIBS_DIR}/libgtest_main.a
      )

target_link_libraries(${project_test_name} ${CMAKE_THREAD_LIBS_INIT})

add_test(test1 ${project_test_name})
