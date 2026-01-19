cmake_minimum_required(VERSION 3.0.2)
project(gazebo_map_creator_interface)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
)

add_service_files(
  FILES
    MapRequest.srv
)

generate_messages(
  DEPENDENCIES
    geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs message_runtime
)

install(DIRECTORY srv
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
