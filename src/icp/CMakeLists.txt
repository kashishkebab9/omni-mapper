cmake_minimum_required(VERSION 3.0.2)
project(icp)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  visualization_msgs
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})
#add_executable(icp src/icp_solver.cpp)
#target_link_libraries(icp ${catkin_LIBRARIES})
add_executable(icp_retry src/icp_solver_retry.cpp)
target_link_libraries(icp_retry ${catkin_LIBRARIES})
