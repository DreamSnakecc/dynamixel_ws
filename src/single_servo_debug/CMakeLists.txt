cmake_minimum_required(VERSION 3.0.2)
project(single_servo_debug)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  dynamixel_sdk
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/single_servo_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
