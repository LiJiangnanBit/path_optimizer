## Get the directory path of the <target>.cmake file
get_filename_component(path_optimizer_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" DIRECTORY)
message("path_optimizer_CMAKE_DIR: ${path_optimizer_CMAKE_DIR}")

## Add the dependencies of our library
include(CMakeFindDependencyMacro)

find_dependency(IPOPT REQUIRED)
find_dependency(benchmark REQUIRED)
find_dependency(OsqpEigen REQUIRED)
find_dependency(Eigen3 REQUIRED)
find_dependency(OpenCV 3 REQUIRED)
find_dependency(grid_map_core REQUIRED)
find_dependency(grid_map_cv REQUIRED)

## Import the targets
if(NOT TARGET path_optimizer::path_optimizer)
    include("${path_optimizer_CMAKE_DIR}/path_optimizerConfig.cmake")
endif()