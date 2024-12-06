cmake_minimum_required(VERSION 3.0.2)
project(demo)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_transport
)

## Uncomment if you have system dependencies
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs geometry_msgs cv_bridge image_transport
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Ensure the Python script is executable
catkin_install_python(PROGRAMS
  src/line_follower_real.py  # Adjusted to match your folder structure
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
