cmake_minimum_required(VERSION 2.8.3)
project(thormang3_direct_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  control_msgs
  sensor_msgs
  std_msgs
  geometry_msgs
  cmake_modules
  robotis_device
  robotis_framework_common
)

find_package(Eigen REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  roscpp
  control_msgs
  sensor_msgs
  std_msgs
  geometry_msgs
)

add_executable(direct_control_node 
	src/DirectControl.cpp
)

add_dependencies(direct_control_node main_generate_messages_cpp)

target_link_libraries(direct_control_node
  robotis_device
  ${catkin_LIBRARIES}
  ${ROSRT_PLATFORM_LDFLAGS}
)
