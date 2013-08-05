cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})


add_library(darwin_plugin SHARED darwin_plugin.cc)
target_link_libraries(darwin_plugin ${GAZEBO_LIBRARIES})
