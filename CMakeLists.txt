cmake_minimum_required(VERSION 2.8.3)
project(path_optimizer)

## Compile as C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)
add_definitions(-fPIC)
set(CMAKE_CXX_FLAGS " -Wall -Wextra ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

set(catkin_deps
        opt_utils
        grid_map_ros
        internal_grid_map
        car_model
        tinyspline_ros
        )

find_package(catkin REQUIRED COMPONENTS
        ${catkin_deps}
        )

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")

message(STATUS "CMAKE SOURCE DIR:" ${CMAKE_MODULE_PATH})

## find system depends libraries
find_package(IPOPT REQUIRED)
find_package(benchmark REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
set(Clothoids_INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/libClothoids/include")
set(Clothoids_LIBRARIES "${CMAKE_SOURCE_DIR}/libClothoids/lib/libClothoids_linux.so")
catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS ${catkin_deps}
        DEPENDS Clothoids
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${Clothoids_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
)

# Declare a C++ library
add_library(${PROJECT_NAME}
        src/tools/spline.cpp
        src/path_optimizer.cpp
        src/reference_path_smoother/reference_path_smoother.cpp
        src/tools/collision_checker.cpp
        src/osqp_related.cpp
        src/tools/tools.cpp

        )
target_link_libraries(${PROJECT_NAME} glog ${IPOPT_LIBRARIES} ${catkin_LIBRARIES} ${Clothoids_LIBRARIES} OsqpEigen::OsqpEigen osqp::osqp
        )

add_executable(${PROJECT_NAME}_benchmark
        src/path_optimizer_benchmark.cpp
        )
target_link_libraries(${PROJECT_NAME}_benchmark ${PROJECT_NAME} glog ${IPOPT_LIBRARIES} ${catkin_LIBRARIES} benchmark::benchmark ${Clothoids_LIBRARIES} OsqpEigen::OsqpEigen osqp::osqp
        )