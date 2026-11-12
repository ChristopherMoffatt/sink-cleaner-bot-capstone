cmake_minimum_required(VERSION 3.0.2)
project(ros1_apriltag)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_transport
  tf
  tf2_ros
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/apriltag_node.py
  scripts/camera_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
