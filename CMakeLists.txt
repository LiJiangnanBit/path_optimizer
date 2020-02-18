cmake_minimum_required(VERSION 2.8.3)
project(path_optimizer)

## Compile as C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11 -w)
set(CMAKE_CXX_FLAGS " -Wall -Wextra ${CMAKE_CXX_FLAGS}")
set(CMAKE_BUILD_TYPE "Release")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")
#set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "${CMAKE_SOURCE_DIR}/external")
set(EXTERNAL_INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/external/include")
set(EXTERNAL_LIB_DIRS "${CMAKE_SOURCE_DIR}/external/lib")

find_package(IPOPT REQUIRED)
find_package(benchmark REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_cv REQUIRED)

include_directories(
        include
        ${EXTERNAL_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${grid_map_core_INCLUDE_DIRS}
        ${grid_map_cv_INCLUDE_DIRS}

)

add_library(${PROJECT_NAME}
        src/tools/tools.cpp
        src/tools/spline.cpp
        src/path_optimizer/path_optimizer.cpp
        src/reference_path_smoother/frenet_reference_path_smoother.cpp
        src/tools/collision_checker.cpp
        src/path_optimizer/solver_interface.cpp
        src/reference_path_smoother/cartesian_reference_path_smoother.cpp
        src/reference_path_smoother/reference_path_smoother.cpp src/tools/Map.cpp include/tools/Map.hpp src/tools/car_geometry.cpp)
target_link_libraries(${PROJECT_NAME} glog ${IPOPT_LIBRARIES} OsqpEigen::OsqpEigen osqp::osqp
        ${EXTERNAL_LIB_DIRS}/libtinyspline.so ${EXTERNAL_LIB_DIRS}/libtinysplinecpp.so ${grid_map_core_LIBRARIES}
        ${grid_map_cv_LIBRARIES}
        )

add_executable(${PROJECT_NAME}_benchmark
        src/test/path_optimizer_benchmark.cpp
        )
target_link_libraries(${PROJECT_NAME}_benchmark
        ${PROJECT_NAME} benchmark::benchmark
        )

