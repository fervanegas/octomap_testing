cmake_minimum_required(VERSION 3.8)
project(octomap_testing)

## Compile as C++14, supported by ROS 2 Humble
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


## Find ROS 2 macros and libraries
## ROS2 CHANGE: catkin has been replaced by ament_cmake
find_package(ament_cmake REQUIRED)

## ROS 2 CHANGE: roscpp has been replaced by rclcpp
find_package(rclcpp REQUIRED)

## OctoMap dependency remains unchanged
find_package(octomap REQUIRED)


###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  ${OCTOMAP_INCLUDE_DIRS}
)

## Declare the C++ executable
## The original executable name has been retained
add_executable(
  octomap_creator_node
  src/octomap_creator.cpp
)

## ROS 2 CHANGE:
## Connect the executable to its ROS 2 dependencies
ament_target_dependencies(
  octomap_creator_node
  rclcpp
)

## Link the OctoMap libraries
target_link_libraries(
  octomap_creator_node
  ${OCTOMAP_LIBRARIES}
)


#############
## Install ##
#############

## ROS 2 CHANGE:
## Install the executable so that it can be found by ros2 run and ros2 launch
install(
  TARGETS octomap_creator_node
  DESTINATION lib/${PROJECT_NAME}
)

## ROS 2 CHANGE:
## Install the launch files, maps and RViz configuration into the package
## share directory
install(
  DIRECTORY
    launch
    maps
    rviz
  DESTINATION share/${PROJECT_NAME}
)


## ROS 2 CHANGE:
## Complete the ament package
ament_package()