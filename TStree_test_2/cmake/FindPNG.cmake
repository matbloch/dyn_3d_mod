#
# Try to find AntTweakBar library and include path.
# Once done this will define
#
# PNG_FOUND
# PNG_INCLUDE_DIR
# PNG_LIBRARY
#

FIND_PATH(PNG_INCLUDE_DIR png.h
      PATHS
	    ${LIBIGL_INCLUDE_DIR}/../external/libpng/
      ${PROJECT_SOURCE_DIR}/../../external/libpng/
      ${PROJECT_SOURCE_DIR}/../external/libpng/
			${PROJECT_SOURCE_DIR}/../libigl/external/libpng/
      /usr/local/include
      /usr/X11/include
      /usr/include
      /opt/local/include
      NO_DEFAULT_PATH)

FIND_LIBRARY( PNG_LIBRARY png
  PATHS
		${LIBIGL_INCLUDE_DIR}/../external/libpng/build
    ${PROJECT_SOURCE_DIR}/../../external/libpng/build
    ${PROJECT_SOURCE_DIR}/../external/libpng/build
		${PROJECT_SOURCE_DIR}/../libigl/external/libpng/build
    /opt/local/lib
    /usr/local
    /usr/X11
    /usr
  PATH_SUFFIXES
    a
    lib64
    lib
    dylib
    NO_DEFAULT_PATH
)

# message(FATAL_ERROR ${PNG_LIBRARY})

if(PNG_INCLUDE_DIR AND PNG_LIBRARY)
	message(STATUS "Found PNG: ${PNG_INCLUDE_DIR}")
else(PNG_INCLUDE_DIR AND PNG_LIBRARY)
	message(FATAL_ERROR "could NOT find PNG")
endif(PNG_INCLUDE_DIR AND PNG_LIBRARY)
