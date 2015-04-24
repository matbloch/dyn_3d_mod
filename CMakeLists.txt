set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)




rosbuild_init()


#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib_out)




#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
#rosbuild_add_executable(tree src/tree_tests.cpp)
#rosbuild_add_executable(stream src/stream_depth.cpp)


# add boost libraries
#rosbuild_add_boost_directories()

# pcl
find_package( PCL 1.2 REQUIRED segmentation filters visualization )

# eigen lib
find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

### add header files
include_directories(${PROJECT_SOURCE_DIR}/lib)	# use #include "myfile.h". With sub-folder of lib: #include "mylib/myfile.h"

# add executables
rosbuild_add_executable(main src/main.cpp)
rosbuild_link_boost(main program_options regex)

### EXTRINSICS CALIBRATION
rosbuild_add_executable(ext src/extrinsics_calibration.cpp)
target_link_libraries(ext ${PCL_LIBRARIES} libvtkCommon.so libvtkFiltering.so ) # for pcl::visualizer

### TESTS
rosbuild_add_executable(test_config test/test_config.cpp)
rosbuild_add_executable(test_filters test/test_filters.cpp)

