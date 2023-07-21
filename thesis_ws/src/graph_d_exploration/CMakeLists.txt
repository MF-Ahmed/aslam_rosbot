cmake_minimum_required(VERSION 2.8.3)
project(graph_d_exploration)

add_compile_options(-std=c++11)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
  visualization_msgs
  message_generation
  dynamic_reconfigure
  actionlib_msgs
)

add_message_files(
  DIRECTORY
  msg
  FILES
  Point2D.msg
  PointArray.msg
  InfoMatrix.msg
  BoolArray.msg
)

add_action_files(
  DIRECTORY
  action
  FILES
  MergePoints.action
  ChooseGoal.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
)

generate_dynamic_reconfigure_options(
  cfg/informationGain.cfg
)

catkin_package(

CATKIN_DEPENDS message_runtime
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(d_opt_global_rrt_detector src/global_rrt_detector.cpp src/functions.cpp src/mtrand.cpp)
target_link_libraries(d_opt_global_rrt_detector ${catkin_LIBRARIES})

add_executable(d_opt_local_rrt_detector src/local_rrt_detector.cpp src/functions.cpp src/mtrand.cpp)
target_link_libraries(d_opt_local_rrt_detector ${catkin_LIBRARIES})
