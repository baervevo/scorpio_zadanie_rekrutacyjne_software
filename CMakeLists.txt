cmake_minimum_required(VERSION 3.0.2)
project(autonomy_simulator)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  std_msgs
  roscpp
)

add_service_files(
  FILES
  GetMap.srv
)

add_message_files(
  FILES
  RoverMap.msg
  RoverPose.msg
  SetGoal.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(autonomy_simulator
  src/main.cpp
  src/autonomy_simulator.cpp
  src/map_generation.cpp
)

add_executable(path_finder
  src/path_finder_manager.cpp
  src/known_map_path_finder.cpp
  src/unknown_map_path_finder.cpp
  src/path_finder.cpp
)

add_dependencies(autonomy_simulator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(map_generation src/map_generation.cpp)

target_link_libraries(autonomy_simulator ${catkin_LIBRARIES} map_generation)

target_link_libraries(path_finder ${catkin_LIBRARIES} )

add_subdirectory(test)