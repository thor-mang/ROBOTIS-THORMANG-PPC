cmake_minimum_required(VERSION 2.8.3)
project(thormang3_sensors)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  laser_assembler
  pcl_ros
)

find_package(PCL 1.7 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES my_util
#  CATKIN_DEPENDS laser_assembler
#  DEPENDS system_lib
#  pcl_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
)

add_definitions(
  ${PCL_DEFINITIONS}
)

add_executable(assemble_laser_node src/assemble_laser.cpp)

add_dependencies(assemble_laser_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(assemble_laser_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

