cmake_minimum_required(VERSION 3.0.2)
project(ekf_implementation)

add_compile_options(-std=c++14)
add_definitions(-DBOOST_BIND_GLOBAL_PLACEHOLDERS)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(motion_model src/motion_command.cpp)
target_link_libraries(motion_model ${catkin_LIBRARIES})
