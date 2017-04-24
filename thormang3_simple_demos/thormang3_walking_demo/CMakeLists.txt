cmake_minimum_required(VERSION 2.8.3)
project(thormang3_walking_demo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  robotis_controller_msgs
  thormang3_walking_module_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  roscpp
  std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(thormang3_walking_demo
   src/thormang3_walking_demo.cpp
   src/main.cpp
)

add_dependencies(thormang3_walking_demo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(thormang3_walking_demo
  yaml-cpp
  ${catkin_LIBRARIES}
)

