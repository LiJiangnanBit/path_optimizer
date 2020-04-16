//
// Created by ljn on 19-8-24.
//

#include <iostream>
#include <benchmark/benchmark.h>
#include <ros/package.h>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_cv/grid_map_cv.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include "eigen3/Eigen/Dense"
#include "opencv2/core/core.hpp"
#include "opencv2/core/eigen.hpp"
#include "opencv2/opencv.hpp"
#include <vector>
#include <opencv/cv.hpp>
#include "glog/logging.h"
#include <path_optimizer/path_optimizer.hpp>
#include "path_optimizer/tools/eigen2cv.hpp"
#include "path_optimizer/data_struct/data_struct.hpp"

static void BM_optimizePath(benchmark::State &state) {
    // Initialize grid map from image.
    std::string image_dir = ros::package::getPath("path_optimizer");
    std::string base_dir = image_dir;
    std::string image_file = "obstacles_for_benchmark.png";
    image_dir.append("/" + image_file);
    cv::Mat img_src = cv::imread(image_dir, CV_8UC1);
    double resolution = 0.2;  // in meter
    grid_map::GridMap grid_map(std::vector<std::string>{"obstacle", "distance"});
    grid_map::GridMapCvConverter::initializeFromImage(
        img_src, resolution, grid_map, grid_map::Position::Zero());
    // Add obstacle layer.
    unsigned char OCCUPY = 0;
    unsigned char FREE = 255;
    grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 1>(
        img_src, "obstacle", grid_map, OCCUPY, FREE, 0.5);
    // Update distance layer.
    Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> binary =
        grid_map.get("obstacle").cast<unsigned char>();
    cv::distanceTransform(eigen2cv(binary), eigen2cv(grid_map.get("distance")),
                          CV_DIST_L2, CV_DIST_MASK_PRECISE);
    grid_map.get("distance") *= resolution;
    grid_map.setFrameId("/map");

    // Input reference path.
    std::vector<double> x_list_ =
        {36.933, 35.664, 34.5232, 33.5006, 32.5863, 31.7711, 31.0461, 30.4029, 29.8334, 29.33, 28.8857, 28.4938,
         28.1478, 27.8421, 27.5711, 27.3299, 27.1139, 26.919, 26.7415, 26.5781, 26.4261, 26.283, 26.1468, 26.016,
         25.8895, 25.7666, 25.6471, 25.5308, 25.4176, 25.3073, 25.1998, 25.0951, 24.9929, 24.8933, 24.7961, 24.7011,
         24.6084, 24.5178, 24.4292, 24.3425, 24.2578, 24.1748, 24.0936, 24.0141, 23.9361, 23.8597, 23.7848, 23.7114,
         23.6394, 23.5687, 23.4994, 23.4314, 23.3647, 23.2992, 23.235, 23.172, 23.1101, 23.0493, 22.9897, 22.9312,
         22.8738, 22.8174, 22.762, 22.7076, 22.6542, 22.6018, 22.5504, 22.4998, 22.4502, 22.4015, 22.3536, 22.3066,
         22.2605, 22.2151, 22.1707, 22.127, 22.0841, 22.042, 22.0007, 21.9603, 21.9208, 21.8821, 21.8445, 21.8079,
         21.7724, 21.7381, 21.7051, 21.6736, 21.6436, 21.6153, 21.5888, 21.5642, 21.5418, 21.5217, 21.5042, 21.4893,
         21.4773, 21.4685, 21.463, 21.4611};
    std::vector<double> y_list_ =
        {33.6609, 30.1924, 27.1101, 24.3825, 21.9795, 19.8724, 18.0336, 16.437, 15.0581, 13.8733, 12.8606, 11.9994,
         11.2702, 10.6552, 10.1376, 9.70216, 9.3349, 9.02324, 8.7559, 8.52298, 8.31592, 8.1275, 7.95186, 7.78447,
         7.62217, 7.46313, 7.30673, 7.15283, 7.00127, 6.85193, 6.70466, 6.55933, 6.41578, 6.27389, 6.13352, 5.99451,
         5.85674, 5.72006, 5.58434, 5.44943, 5.31518, 5.18147, 5.04815, 4.91508, 4.78211, 4.64912, 4.51595, 4.38246,
         4.24852, 4.11398, 3.9787, 3.84254, 3.70538, 3.5671, 3.4276, 3.28681, 3.14465, 3.00106, 2.85602, 2.70948,
         2.56145, 2.41193, 2.26093, 2.10849, 1.95465, 1.79949, 1.64306, 1.48548, 1.32684, 1.16726, 1.00687,
         0.845838, 0.684314, 0.522481, 0.360532, 0.198675, 0.0371402, -0.123809, -0.283872, -0.442713, -0.599958,
         -0.755201, -0.907996, -1.05786, -1.20428, -1.3467, -1.48454, -1.61716, -1.7439, -1.86408, -1.97694,
         -2.08173, -2.17764, -2.26383, -2.33941, -2.40347, -2.45507, -2.49321, -2.51688, -2.52501};
    std::vector<PathOptimizationNS::State> points, final_path;
    for (size_t i = 0; i != x_list_.size(); ++i) {
        PathOptimizationNS::State state;
        state.x = x_list_[i];
        state.y = y_list_[i];
        points.push_back(state);
    }
    PathOptimizationNS::State start_state, goal_state;
    start_state.x = 36.933;
    start_state.y = 33.6609;
    start_state.z = -1.36375;
    start_state.k = 0;
    goal_state.x = 21.4611;
    goal_state.y = -2.52501;
    goal_state.z = -1.30825;
    goal_state.k = 0;
    for (auto _:state) {
        FLAGS_enable_computation_time_output = false;
        PathOptimizationNS::PathOptimizer path_optimizer(start_state, goal_state, grid_map);
        path_optimizer.solve(points, &final_path);
    }
}
BENCHMARK(BM_optimizePath)->Unit(benchmark::kMillisecond);

static void BM_optimizePathWithoutSmoothing(benchmark::State &state) {
    // Initialize grid map from image.
    std::string image_dir = ros::package::getPath("path_optimizer");
    std::string base_dir = image_dir;
    std::string image_file = "obstacles_for_benchmark.png";
    image_dir.append("/" + image_file);
    cv::Mat img_src = cv::imread(image_dir, CV_8UC1);
    double resolution = 0.2;  // in meter
    grid_map::GridMap grid_map(std::vector<std::string>{"obstacle", "distance"});
    grid_map::GridMapCvConverter::initializeFromImage(
        img_src, resolution, grid_map, grid_map::Position::Zero());
    // Add obstacle layer.
    unsigned char OCCUPY = 0;
    unsigned char FREE = 255;
    grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 1>(
        img_src, "obstacle", grid_map, OCCUPY, FREE, 0.5);
    // Update distance layer.
    Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> binary =
        grid_map.get("obstacle").cast<unsigned char>();
    cv::distanceTransform(eigen2cv(binary), eigen2cv(grid_map.get("distance")),
                          CV_DIST_L2, CV_DIST_MASK_PRECISE);
    grid_map.get("distance") *= resolution;
    grid_map.setFrameId("/map");

    // Input reference path.
    std::vector<double> x_list_ =
        {36.933, 35.664, 34.5232, 33.5006, 32.5863, 31.7711, 31.0461, 30.4029, 29.8334, 29.33, 28.8857, 28.4938,
         28.1478, 27.8421, 27.5711, 27.3299, 27.1139, 26.919, 26.7415, 26.5781, 26.4261, 26.283, 26.1468, 26.016,
         25.8895, 25.7666, 25.6471, 25.5308, 25.4176, 25.3073, 25.1998, 25.0951, 24.9929, 24.8933, 24.7961, 24.7011,
         24.6084, 24.5178, 24.4292, 24.3425, 24.2578, 24.1748, 24.0936, 24.0141, 23.9361, 23.8597, 23.7848, 23.7114,
         23.6394, 23.5687, 23.4994, 23.4314, 23.3647, 23.2992, 23.235, 23.172, 23.1101, 23.0493, 22.9897, 22.9312,
         22.8738, 22.8174, 22.762, 22.7076, 22.6542, 22.6018, 22.5504, 22.4998, 22.4502, 22.4015, 22.3536, 22.3066,
         22.2605, 22.2151, 22.1707, 22.127, 22.0841, 22.042, 22.0007, 21.9603, 21.9208, 21.8821, 21.8445, 21.8079,
         21.7724, 21.7381, 21.7051, 21.6736, 21.6436, 21.6153, 21.5888, 21.5642, 21.5418, 21.5217, 21.5042, 21.4893,
         21.4773, 21.4685, 21.463, 21.4611};
    std::vector<double> y_list_ =
        {33.6609, 30.1924, 27.1101, 24.3825, 21.9795, 19.8724, 18.0336, 16.437, 15.0581, 13.8733, 12.8606, 11.9994,
         11.2702, 10.6552, 10.1376, 9.70216, 9.3349, 9.02324, 8.7559, 8.52298, 8.31592, 8.1275, 7.95186, 7.78447,
         7.62217, 7.46313, 7.30673, 7.15283, 7.00127, 6.85193, 6.70466, 6.55933, 6.41578, 6.27389, 6.13352, 5.99451,
         5.85674, 5.72006, 5.58434, 5.44943, 5.31518, 5.18147, 5.04815, 4.91508, 4.78211, 4.64912, 4.51595, 4.38246,
         4.24852, 4.11398, 3.9787, 3.84254, 3.70538, 3.5671, 3.4276, 3.28681, 3.14465, 3.00106, 2.85602, 2.70948,
         2.56145, 2.41193, 2.26093, 2.10849, 1.95465, 1.79949, 1.64306, 1.48548, 1.32684, 1.16726, 1.00687,
         0.845838, 0.684314, 0.522481, 0.360532, 0.198675, 0.0371402, -0.123809, -0.283872, -0.442713, -0.599958,
         -0.755201, -0.907996, -1.05786, -1.20428, -1.3467, -1.48454, -1.61716, -1.7439, -1.86408, -1.97694,
         -2.08173, -2.17764, -2.26383, -2.33941, -2.40347, -2.45507, -2.49321, -2.51688, -2.52501};
    std::vector<PathOptimizationNS::State> points, optimized_path, final_path;
    for (size_t i = 0; i != x_list_.size(); ++i) {
        PathOptimizationNS::State state;
        state.x = x_list_[i];
        state.y = y_list_[i];
        points.push_back(state);
    }
    PathOptimizationNS::State start_state, goal_state;
    start_state.x = 36.933;
    start_state.y = 33.6609;
    start_state.z = -1.36375;
    start_state.k = 0;
    goal_state.x = 21.4611;
    goal_state.y = -2.52501;
    goal_state.z = -1.30825;
    goal_state.k = 0;

    PathOptimizationNS::PathOptimizer path_optimizer(start_state, goal_state, grid_map);
    path_optimizer.solve(points, &optimized_path);
    for (auto _:state) {
        FLAGS_enable_computation_time_output = false;
        path_optimizer.solveWithoutSmoothing(optimized_path, &final_path);
    }
}
BENCHMARK(BM_optimizePathWithoutSmoothing)->Unit(benchmark::kMillisecond);

BENCHMARK_MAIN();