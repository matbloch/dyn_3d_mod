cmake_minimum_required(VERSION 2.8)
project(TStree)

include("CMakeLists.shared")

find_package( OpenCV REQUIRED )
#find_package( OpenGL REQUIRED)
#find_package( GLUT REQUIRED)
add_definitions(-DIGL_NO_OPENGL)

file(GLOB SOURCES
    "${PROJECT_SOURCE_DIR}/src/*.h"
    "${PROJECT_SOURCE_DIR}/src/*.cpp"
    "${PROJECT_SOURCE_DIR}/src/*.hpp"
)

LIST(APPEND SOURCES ${YIMAGE_SOURCES})
# LIST(APPEND SHARED_LIBRARIES ${PNG_LIBRARY})

#message(FATAL_ERROR ${PNG_LIBRARY})

add_executable(${PROJECT_NAME}_bin ${SOURCES} ${SHARED_SOURCES})

# for mac
#target_link_libraries(${PROJECT_NAME}_bin ${SHARED_LIBRARIES} ${PNG_LIBRARY} ${EMBREE_LIBRARIES} ${OpenCV_LIBS})

# for linux
target_link_libraries(${PROJECT_NAME}_bin ${SHARED_LIBRARIES} ${PNG_LIBRARY} ${EMBREE_LIBRARIES} Xcursor Xinerama)

