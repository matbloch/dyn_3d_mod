cmake_minimum_required(VERSION 2.6)
project(ex4)

include("CMakeLists.shared")

add_definitions(-DIGL_NO_OPENGL)

file(GLOB SOURCES
    "${PROJECT_SOURCE_DIR}/src/*.h"
    "${PROJECT_SOURCE_DIR}/src/*.cpp"
)

LIST(APPEND SOURCES ${YIMAGE_SOURCES})
# LIST(APPEND SHARED_LIBRARIES ${PNG_LIBRARY})

#message(FATAL_ERROR ${PNG_LIBRARY})

add_executable(${PROJECT_NAME}_bin ${SOURCES} ${SHARED_SOURCES})
target_link_libraries(${PROJECT_NAME}_bin ${SHARED_LIBRARIES} ${PNG_LIBRARY} ${EMBREE_LIBRARIES})
