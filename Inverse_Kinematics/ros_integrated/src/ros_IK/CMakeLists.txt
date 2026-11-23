cmake_minimum_required(VERSION 3.0.2)
project(arm_ik)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(ik_node src/simpler.cpp)
target_link_libraries(ik_node ${catkin_LIBRARIES})

add_executable(algorithm_node src/algorithm.cpp)
target_link_libraries(algorithm_node ${catkin_LIBRARIES})
