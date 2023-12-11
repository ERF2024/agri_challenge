cmake_minimum_required(VERSION 2.8.3)
project(agri_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  trajectory_msgs
  moveit_core
)

catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/agri_control.cpp src/agri_control_node.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(agri_state_machine src/agri_state_machine.cpp src/agri_state_machine_node.cpp)
target_link_libraries(agri_state_machine ${catkin_LIBRARIES})
