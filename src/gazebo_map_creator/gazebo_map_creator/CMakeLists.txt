cmake_minimum_required(VERSION 3.0.2)
project(gazebo_map_creator)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  gazebo_dev
  gazebo_map_creator_interface
  gazebo_ros
  geometry_msgs
  roscpp
)

find_package(gazebo REQUIRED)
find_package(PCL 1.2 REQUIRED)
find_package(octomap REQUIRED)

# catkin_package(DEPENDS ...) expects <name>_INCLUDE_DIRS / <name>_LIBRARIES.
set(gazebo_INCLUDE_DIRS ${GAZEBO_INCLUDE_DIRS})
set(gazebo_LIBRARIES ${GAZEBO_LIBRARIES})
set(octomap_INCLUDE_DIRS ${OCTOMAP_INCLUDE_DIRS})
set(octomap_LIBRARIES ${OCTOMAP_LIBRARIES})

catkin_package(
  CATKIN_DEPENDS
    gazebo_map_creator_interface
    gazebo_ros
    geometry_msgs
    roscpp
  DEPENDS
    gazebo
    octomap
    PCL
  LIBRARIES
    gazebo_map_creator
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${PCL_LIBRARY_DIRS}
)

add_definitions(${PCL_DEFINITIONS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(gazebo_map_creator SHARED
  src/gazbeo_map_creator.cpp
)

add_dependencies(gazebo_map_creator
  ${catkin_EXPORTED_TARGETS}
  ${gazebo_map_creator_interface_EXPORTED_TARGETS}
)

target_link_libraries(gazebo_map_creator
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  ${PCL_LIBRARIES}
)

install(TARGETS gazebo_map_creator
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/request_map.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
