cmake_minimum_required(VERSION 3.0.2)
project(Devices)
find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
)
catkin_package()
include_directories(
  ${catkin_INCLUDE_DIRS}
)
catkin_install_python(PROGRAMS
  joy.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
