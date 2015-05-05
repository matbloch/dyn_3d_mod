# - Try to find the TRIANGLE library
# Once done this will define
#
#  TRIANGLE_FOUND - system has TRIANGLE
#  TRIANGLE_INCLUDE_DIR - the TRIANGLE include directory
#  TRIANGLE_SOURCES - the TRIANGLE source files

IF (WIN32)
   add_definitions(-DNO_TIMER)
ENDIF (WIN32)

FIND_PATH(TRIANGLE_INCLUDE_DIR triangle.h
   /usr/include
   /usr/local/include
   ${PROJECT_SOURCE_DIR}/../libigl/external/triangle/
   ${PROJECT_SOURCE_DIR}/../../external/triangle/
   NO_DEFAULT_PATH
)

set(TRIANGLE_SOURCES ${TRIANGLE_INCLUDE_DIR}/triangle.c)

if(TRIANGLE_INCLUDE_DIR)
   message(STATUS "Found TRIANGLE: ${TRIANGLE_INCLUDE_DIR}")
else(TRIANGLE_INCLUDE_DIR)
   message(FATAL_ERROR "could NOT find TRIANGLE")
endif(TRIANGLE_INCLUDE_DIR)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DTRILIBRARY -DANSI_DECLARATORS")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DTRILIBRARY -DANSI_DECLARATORS")

MARK_AS_ADVANCED(TRIANGLE_INCLUDE_DIR TRIANGLE_LIBRARIES TRIANGLE_SOURCES)
