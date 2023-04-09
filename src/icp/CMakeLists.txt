cmake_minimum_required(VERSION 3.0.2)
project(icp)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  visualization_msgs
  laser_geometry
  tf2
  tf2_ros
  tf2_geometry_msgs

)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(icp_retry src/icp_solver_node.cpp)
target_link_libraries(icp_retry ${catkin_LIBRARIES})
add_executable(scan_converter src/scan_to_pcl.cpp)
target_link_libraries(scan_converter ${catkin_LIBRARIES})
