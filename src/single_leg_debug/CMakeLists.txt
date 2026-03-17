cmake_minimum_required(VERSION 3.0.2)
project(single_leg_debug)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  dynamixel_sdk
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/leg_calibration.py
  scripts/walking_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
