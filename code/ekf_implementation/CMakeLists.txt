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
  nav_msgs
  message_generation
)

add_message_files(
  FILES
  encoder.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(CATKIN_DEPENDS message_generation)

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(ekf src/ekf_node.cpp)
target_link_libraries(ekf ${catkin_LIBRARIES})

add_executable(sim_enc src/simulated_encoder.cpp)
target_link_libraries(sim_enc ${catkin_LIBRARIES})
