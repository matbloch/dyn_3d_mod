cmake_minimum_required(VERSION 2.8)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

rosbuild_init()

########################################################
##		PATH SETUP
########################################################

SET(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
set(CMAKE_COLOR_MAKEFILE ON)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

### add header files
# usage:
# include "myfile.h". With sub-folder of lib: 
# include "mylib/myfile.h"

include_directories(${PROJECT_SOURCE_DIR}/lib)	

########################################################
##		3RD PARTY LIBRARIES
########################################################

find_package( OpenCV REQUIRED )
add_definitions(-DIGL_NO_OPENGL)

# pcl
find_package( PCL 1.2 REQUIRED segmentation filters visualization )

# eigen
include_directories(${Eigen_INCLUDE_DIRS})

########################################################
##		LIB IGL
########################################################

include("CMakeLists.shared")
add_definitions(-DIGL_NO_OPENGL)
LIST(APPEND SOURCES ${YIMAGE_SOURCES})

########################################################
##		EXECUTABLES
########################################################

# EXAMPLE:
#add_executable(${PROJECT_NAME}_bin ${SOURCES} ${SHARED_SOURCES})
#target_link_libraries(${PROJECT_NAME}_bin ${SHARED_LIBRARIES} ${PNG_LIBRARY} ${EMBREE_LIBRARIES} ${OpenCV_LIBS} Xcursor Xinerama)

### MAIN FILE
rosbuild_add_executable(start_recording src/recording_node.cpp ${SHARED_SOURCES})
target_link_libraries(start_recording ${SHARED_LIBRARIES} ${PNG_LIBRARY} ${EMBREE_LIBRARIES} ${OpenCV_LIBS} Xcursor Xinerama)

### EXTRINSICS CALIBRATION
rosbuild_add_executable(start_calibration src/extrinsics_calibration.cpp)
target_link_libraries(start_calibration ${PCL_LIBRARIES} libvtkCommon.so libvtkFiltering.so ) # for pcl::visualizer

### TOOLS
rosbuild_add_executable(cutoff_tuning src/cutoff_tuning.cpp)
target_link_libraries(cutoff_tuning ${PCL_LIBRARIES} libvtkCommon.so libvtkFiltering.so ) # for pcl::visualizer

### TESTS
#rosbuild_add_executable(TStree test/TStree_test.cpp ${SHARED_SOURCES})
#target_link_libraries(TStree ${SHARED_LIBRARIES} ${PNG_LIBRARY} ${EMBREE_LIBRARIES} ${OpenCV_LIBS} #Xcursor Xinerama)

### DEMOS
#rosbuild_add_executable(show_point_clouds demo/show_point_clouds.cpp ${SHARED_SOURCES})
#target_link_libraries(show_point_clouds ${PCL_LIBRARIES} libvtkCommon.so libvtkFiltering.so ) # for pcl::visualizer
#rosbuild_add_executable(show_streams demo/show_streams.cpp ${SHARED_SOURCES})
#target_link_libraries(show_streams ${PCL_LIBRARIES} libvtkCommon.so libvtkFiltering.so ) # for pcl::visualizer


