cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(Qt4 REQUIRED)
include(${QT_USE_FILE})
include_directories(${CMAKE_BINARY_DIR})
include_directories(${QT_INCLUDE_DIR})

add_definitions(-DQT_NO_KEYWORDS)

qt4_wrap_cpp(MOC_FILES
  include/cat_visualization/teleop_visualization_qt_wrapper.h
)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
rosbuild_add_library(${PROJECT_NAME}
  src/teleop_visualization.cpp
  src/teleop_visualization_qt_wrapper.cpp
  src/cat_visualizer.cpp

  include/cat_visualization/teleop_visualization.h
  include/cat_visualization/teleop_visualization_qt_wrapper.h
  include/cat_visualization/cat_visualizer.h

  ${MOC_FILES}
  )
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES})


rosbuild_add_executable(cat_visualizer_node src/cat_visualizer_node.cpp)
target_link_libraries(cat_visualizer_node ${QT_LIBRARIES} ${PROJECT_NAME})
