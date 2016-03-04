cmake_minimum_required(VERSION 2.8.3)
project(thormang3_manipulation_demo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  robotis_controller_msgs
  thormang3_manipulation_module_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  roscpp
  std_msgs
)

add_executable(thormang3_manipulation_demo 
	src/thormang3_manipulation_demo.cpp
)

target_link_libraries(thormang3_manipulation_demo
  ${catkin_LIBRARIES}
)

