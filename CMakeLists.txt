cmake_minimum_required(VERSION 2.8.3)
project(baxter_haptics)

find_package(catkin
  REQUIRED
  COMPONENTS
  rospy
  baxter_core_msgs
  baxter_interface
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
  rospy
  baxter_interface
  baxter_core_msgs
)

install(
  DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
