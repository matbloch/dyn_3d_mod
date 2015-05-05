# - Try to find the YIMAGE library
# Once done this will define
#
#  YIMAGE_FOUND - system has YIMAGE
#  YIMAGE_INCLUDE_DIR - the YIMAGE include directory
#  YIMAGE_SOURCES - the YIMAGE source files

FIND_PATH(YIMAGE_INCLUDE_DIR YImage.hpp
   /usr/include
   /usr/local/include
   ${PROJECT_SOURCE_DIR}/../libigl/external/yimg/
   ${PROJECT_SOURCE_DIR}/../../external/yimg/
   ${PROJECT_SOURCE_DIR}/../../libigl/external/yimg/
   NO_DEFAULT_PATH
)

set(YIMAGE_SOURCES ${YIMAGE_INCLUDE_DIR}/YImage.cpp)

if(YIMAGE_INCLUDE_DIR)
   message(STATUS "Found YIMAGE: ${YIMAGE_INCLUDE_DIR}")
else(YIMAGE_INCLUDE_DIR)
   message(FATAL_ERROR "could NOT find YIMAGE")
endif(YIMAGE_INCLUDE_DIR)

MARK_AS_ADVANCED(YIMAGE_INCLUDE_DIR YIMAGE_LIBRARIES YIMAGE_SOURCES)
