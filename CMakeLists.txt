cmake_minimum_required(VERSION 2.8.3)
project(adlink_ddsbot)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  costmap_2d
  std_msgs
  geometry_msgs
  message_generation
  dynamic_reconfigure
)

## dynamic reconfigure config
generate_dynamic_reconfigure_options(
  cfg/SwarmLayer.cfg
)

## custom message
add_message_files(
  FILES
  Robot.msg
  MultiRobots.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)


catkin_package(
    CATKIN_DEPENDS std_msgs geometry_msgs message_runtime costmap_2d dynamic_reconfigure
)

## Specify additional locations of header files
include_directories(
  include ${catkin_INCLUDE_DIRS}
)

## add cpp library
add_library(swarm_layers src/swarm_layer.cpp)

## Add cmake target dependencies of the executable/library
add_dependencies(swarm_layers 
${PROJECT_NAME}_gencfg ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)


## Install costmap plugin
install(FILES costmap_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS swarm_layers
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
