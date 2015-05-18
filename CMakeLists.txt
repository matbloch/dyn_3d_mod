cmake_minimum_required(VERSION 2.8)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

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
include_directories(${PROJECT_SOURCE_DIR}/lib)	# use 
#include "myfile.h". With sub-folder of lib: 
#include "mylib/myfile.h"

########################################################
##		3RD PARTY LIBRARIES
########################################################

find_package( OpenCV REQUIRED )
add_definitions(-DIGL_NO_OPENGL)

# pcl
find_package( PCL 1.2 REQUIRED segmentation filters visualization )

# eigen
include_directories(${Eigen_INCLUDE_DIRS})

#message(FATAL_ERROR ${PNG_LIBRARY})

########################################################
##		LIB IGL
########################################################

include("CMakeLists.shared")
add_definitions(-DIGL_NO_OPENGL)
LIST(APPEND SOURCES ${YIMAGE_SOURCES})


########################################################
##		EXECUTABLES
########################################################

#add_executable(${PROJECT_NAME}_bin ${SOURCES} ${SHARED_SOURCES})
#target_link_libraries(${PROJECT_NAME}_bin ${SHARED_LIBRARIES} ${PNG_LIBRARY} ${EMBREE_LIBRARIES} ${OpenCV_LIBS} Xcursor Xinerama)

### MAIN FILES
#rosbuild_add_executable(main src/main.cpp)
#rosbuild_link_boost(main program_options regex)
rosbuild_add_executable(preprocessing_node src/preprocessing_node.cpp ${SHARED_SOURCES})
target_link_libraries(preprocessing_node ${SHARED_LIBRARIES} ${PNG_LIBRARY} ${EMBREE_LIBRARIES} ${OpenCV_LIBS} Xcursor Xinerama)

### EXTRINSICS CALIBRATION
#rosbuild_add_executable(ext src/extrinsics_calibration.cpp)
#target_link_libraries(ext ${PCL_LIBRARIES} libvtkCommon.so libvtkFiltering.so ) # for pcl::visualizer

### TESTS
#rosbuild_add_executable(TStree test/TStree_test.cpp ${SHARED_SOURCES})
#target_link_libraries(TStree ${SHARED_LIBRARIES} ${PNG_LIBRARY} ${EMBREE_LIBRARIES} ${OpenCV_LIBS} #Xcursor Xinerama)

### DEMOS
rosbuild_add_executable(show_point_clouds demo/show_point_clouds.cpp ${SHARED_SOURCES})
target_link_libraries(show_point_clouds ${PCL_LIBRARIES} libvtkCommon.so libvtkFiltering.so ) # for pcl::visualizer
rosbuild_add_executable(show_streams demo/show_streams.cpp ${SHARED_SOURCES})
target_link_libraries(show_streams ${PCL_LIBRARIES} libvtkCommon.so libvtkFiltering.so ) # for pcl::visualizer


