cmake_minimum_required(VERSION 3.0.2)
project(drain_tag_nav)
find_package(catkin REQUIRED)
catkin_package()
catkin_install_python(PROGRAMS scripts/tag_approach_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
