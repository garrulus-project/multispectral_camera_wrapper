cmake_minimum_required(VERSION 3.0.2)
project(multispectral_camera_wrapper)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roslint
)

roslint_python()
catkin_python_setup()
catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

