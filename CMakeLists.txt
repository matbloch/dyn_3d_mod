set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()


#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

########################################################
##		ROS HYDRO ONLY
########################################################

find_package(cmake_modules REQUIRED)		# required for ros indigos

########################################################
##		3RD PARTY LIBRARIES
########################################################

# add boost libraries
#rosbuild_add_boost_directories()

# pcl
find_package( PCL 1.2 REQUIRED segmentation filters visualization )

# eigen
find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

########################################################
##		HEADER FILES
########################################################

### add header files
include_directories(${PROJECT_SOURCE_DIR}/lib)	# use #include "myfile.h". With sub-folder of lib: #include "mylib/myfile.h"


########################################################
##		EXECUTABLES
########################################################

# add executables
#rosbuild_add_executable(main src/main.cpp)
#rosbuild_link_boost(main program_options regex)
rosbuild_add_executable(preprocessing_node src/preprocessing_node.cpp)

### EXTRINSICS CALIBRATION
rosbuild_add_executable(ext src/extrinsics_calibration.cpp)
target_link_libraries(ext ${PCL_LIBRARIES} libvtkCommon.so libvtkFiltering.so ) # for pcl::visualizer

### TESTS
rosbuild_add_executable(test_config test/test_config.cpp)
rosbuild_add_executable(test_filters test/test_filters.cpp)
rosbuild_add_executable(test_voxels test/test_voxels.cpp)
