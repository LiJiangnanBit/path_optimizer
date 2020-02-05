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
        roscpp
        opt_utils
        grid_map_ros
        internal_grid_map
        car_model
        tinyspline_ros
        ros_viz_tools
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
find_package(OpenCV REQUIRED)

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
        DEPENDS OpenCV
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        include
        ${OpenCV_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        ${Clothoids_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
)

# Declare a C++ library
add_library(${PROJECT_NAME}
        src/tools/tools.cpp
        src/tools/spline.cpp
        src/path_optimizer/path_optimizer.cpp
        src/reference_path_smoother/frenet_reference_path_smoother.cpp
        src/tools/collision_checker.cpp
        src/path_optimizer/solver_interface.cpp
        src/reference_path_smoother/cartesian_reference_path_smoother.cpp
        )
target_link_libraries(${PROJECT_NAME} glog ${IPOPT_LIBRARIES} ${catkin_LIBRARIES} ${Clothoids_LIBRARIES} OsqpEigen::OsqpEigen osqp::osqp
        )

add_executable(${PROJECT_NAME}_benchmark
        src/test/path_optimizer_benchmark.cpp
        )
target_link_libraries(${PROJECT_NAME}_benchmark
        ${PROJECT_NAME} benchmark::benchmark
        )

add_executable(${PROJECT_NAME}_demo
        src/test/demo.cpp)
target_link_libraries(${PROJECT_NAME}_demo
        ${PROJECT_NAME} ${OpenCV_LIBRARIES}
        )