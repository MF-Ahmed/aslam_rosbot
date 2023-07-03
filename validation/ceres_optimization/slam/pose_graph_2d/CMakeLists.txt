cmake_minimum_required(VERSION 2.8)
project(ceres-example)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-std=c++14 -O3")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)


# Ceres
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})


add_executable(pose_graph_2d
angle_manifold.h
normalize_angle.h
pose_graph_2d.cc
pose_graph_2d_error_term.h
types.h)

target_link_libraries(pose_graph_2d PRIVATE Ceres::ceres)

