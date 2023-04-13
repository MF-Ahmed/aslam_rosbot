cmake_minimum_required(VERSION 2.8.3)
project(rosbot_mpc)

add_compile_options(-std=c++14)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        nav_msgs
        std_msgs
        geometry_msgs
        roscpp
        rospy
        costmap_converter
        dynamic_reconfigure
        tf
        sensor_msgs
        message_filters
        rosbot_fsm
        )



generate_dynamic_reconfigure_options(
        cfg/NonLinearMPC.cfg
)
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
        INCLUDE_DIRS include cfg
        CATKIN_DEPENDS
        nav_msgs
        std_msgs
        geometry_msgs
        roscpp
        rospy
        costmap_converter
        rosbot_fsm
)


add_library(rosbot_mpc_lib
        src/nlmpc.cpp
        solver_made_from_cpp/qpoases/SRC/Bounds.cpp
        solver_made_from_cpp/qpoases/SRC/Constraints.cpp
        solver_made_from_cpp/qpoases/SRC/CyclingManager.cpp
        solver_made_from_cpp/qpoases/SRC/Indexlist.cpp
        solver_made_from_cpp/qpoases/SRC/MessageHandling.cpp
        solver_made_from_cpp/qpoases/SRC/QProblem.cpp
        solver_made_from_cpp/qpoases/SRC/QProblemB.cpp
        solver_made_from_cpp/qpoases/SRC/SubjectTo.cpp
        solver_made_from_cpp/qpoases/SRC/Utils.cpp
        solver_made_from_cpp/qpoases/SRC/EXTRAS/SolutionAnalysis.cpp
        solver_made_from_cpp/solver/acado_qpoases_interface.cpp
        solver_made_from_cpp/solver/acado_integrator.c
        solver_made_from_cpp/solver/acado_solver.c
        solver_made_from_cpp/solver/acado_auxiliary_functions.c
        )

include_directories(
        include
        ${PROJECT_SOURCE_DIR}/include
        ${catkin_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}/rosbot_mpc
        solver_made_from_cpp/solver
        solver_made_from_cpp/qpoases
        solver_made_from_cpp/qpoases/INCLUDE
        solver_made_from_cpp/qpoases/SRC
        )


###########
## Build ##
###########
## Specify additional locations of header files
## Your package locations should be listed before other locations
install()

# Executables
add_executable(rosbot_mpc src/nlmpc_node.cpp)

# Dependencies
add_dependencies(rosbot_mpc ${${PROJECT_NAME}_EXPORTED_TARGETS})

target_link_libraries(rosbot_mpc_lib ${catkin_LIBRARIES})
# Link libraries
target_link_libraries(rosbot_mpc ${catkin_LIBRARIES} rosbot_mpc_lib)





