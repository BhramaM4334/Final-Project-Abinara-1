cmake_minimum_required(VERSION 3.0.2)
project(lengan_servo)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
  src
  ${catkin_INCLUDE_DIRS}
)

add_executable(stm32_servo_bridge
  src/stm32_servo_bridge.cpp
  src/servo_robot.cpp
)

target_link_libraries(stm32_servo_bridge
  ${catkin_LIBRARIES}
)
