cmake_minimum_required(VERSION 2.8)
project(ceres-example2)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-std=c++14 -O3")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)


# Ceres
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})


add_executable(ceres_optimizer_test ceres_optimizer_test.cpp)

target_link_libraries(ceres_optimizer_test PRIVATE Ceres::ceres)

