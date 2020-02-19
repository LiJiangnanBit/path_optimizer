cmake_minimum_required(VERSION 3.13)
project(path_optimizer)

set(CMAKE_BUILD_TYPE "Release")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")
set(EXTERNAL_LIB_DIRS "${CMAKE_SOURCE_DIR}/external/lib")

find_package(IPOPT REQUIRED)
find_package(benchmark REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(grid_map_core REQUIRED)

add_library(${PROJECT_NAME})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_11)
target_sources(${PROJECT_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/path_optimizer/path_optimizer.cpp>
        PRIVATE
        src/path_optimizer/solver_interface.cpp
        src/reference_path_smoother/cartesian_reference_path_smoother.cpp
        src/reference_path_smoother/reference_path_smoother.cpp
        src/reference_path_smoother/frenet_reference_path_smoother.cpp
        src/tools/tools.cpp
        src/tools/spline.cpp
        src/tools/collision_checker.cpp
        src/tools/Map.cpp
        src/tools/car_geometry.cpp)
target_include_directories(${PROJECT_NAME}
        PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        ${grid_map_core_INCLUDE_DIRS}
        PRIVATE
        ${EIGEN3_INCLUDE_DIR}
        )
target_link_libraries(${PROJECT_NAME}
        PUBLIC
        ${grid_map_core_LIBRARIES}
        osqp::osqp
        OsqpEigen::OsqpEigen
        glog
        PRIVATE
        ${IPOPT_LIBRARIES}
        ${EXTERNAL_LIB_DIRS}/libtinyspline.so
        ${EXTERNAL_LIB_DIRS}/libtinysplinecpp.so
        )

#install
install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}Config
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY include/
        DESTINATION ${CMAKE_INSTALL_PREFIX}/include/path_optimizer)
export(TARGETS
        ${PROJECT_NAME}
        NAMESPACE path_optimizer::
        FILE "${CMAKE_CURRENT_BINARY_DIR}/path_optimizerConfig.cmake"
        )
install(EXPORT
        path_optimizerConfig
        DESTINATION "${CMAKE_INSTALL_DATADIR}/path_optimizer/cmake"
        NAMESPACE path_optimizer::
        )

