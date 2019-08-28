cmake_minimum_required(VERSION 2.8.3)
project(mpc_path_optimizer)

## Compile as C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)
set(CMAKE_CXX_FLAGS "-Wall -Wextra ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

set(catkin_deps
        opt_utils
        grid_map_ros
        internal_grid_map
        )

find_package(catkin REQUIRED COMPONENTS
        ${catkin_deps}
        )

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")

message(STATUS "CMAKE SOURCE DIR:" ${CMAKE_MODULE_PATH})

## find system depends libraries
find_package(IPOPT REQUIRED)
find_package(benchmark REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        #        CATKIN_DEPENDS
        #        DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

# Declare a C++ library
add_library(${PROJECT_NAME}
        src/MpcPathOptimizer.cpp)
target_link_libraries(${PROJECT_NAME} glog ${IPOPT_LIBRARIES} ${catkin_LIBRARIES})

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
add_executable(mpc_path_optimizer_benchmark src/mpc_path_optimizer_benchmark.cpp src/MpcPathOptimizer.cpp)
target_link_libraries(mpc_path_optimizer_benchmark glog ${IPOPT_LIBRARIES} ${catkin_LIBRARIES} benchmark::benchmark)
