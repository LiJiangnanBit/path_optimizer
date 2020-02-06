//
// Created by ljn on 20-2-4.
//

#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <internal_grid_map/internal_grid_map.hpp>
#include <opt_utils/opt_utils.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <ros_viz_tools/ros_viz_tools.h>
#include "path_optimizer/path_optimizer.hpp"

hmpl::State start_state, end_state;
std::vector<hmpl::State> reference_path;
bool start_state_rcv = false, end_state_rcv = false, reference_rcv = false;

void referenceCb(const geometry_msgs::PointStampedConstPtr &p) {
    if (start_state_rcv && end_state_rcv) {
        reference_path.clear();
    }
    hmpl::State reference_point;
    reference_point.x = p->point.x;
    reference_point.y = p->point.y;
    reference_path.emplace_back(reference_point);
    start_state_rcv = end_state_rcv = false;
    reference_rcv = reference_path.size() >= 6;
    std::cout << "received a reference point" << std::endl;
}

void startCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr &start) {
    start_state.x = start->pose.pose.position.x;
    start_state.y = start->pose.pose.position.y;
    start_state.z = tf::getYaw(start->pose.pose.orientation);
    if (reference_rcv) {
        start_state_rcv = true;
    }
    std::cout << "get initial state." << std::endl;
}

void goalCb(const geometry_msgs::PoseStampedConstPtr &goal) {
    end_state.x = goal->pose.position.x;
    end_state.y = goal->pose.position.y;
    end_state.z = tf::getYaw(goal->pose.orientation);
    if (reference_rcv) {
        end_state_rcv = true;
    }
    std::cout << "get the goal." << std::endl;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "path_optimization");
    ros::NodeHandle nh("~");

    // Get map image.
    std::string image_dir = ros::package::getPath("path_optimizer");
    std::string base_dir = image_dir;
    std::string image_file = "gridmap.png";
    image_dir.append("/" + image_file);
    cv::Mat img_src = cv::imread(image_dir, CV_8UC1);
    double resolution = 0.2;  // in meter
    // Initialize map.
    hmpl::InternalGridMap in_gm;
    in_gm.initializeFromImage(img_src,
                              resolution,
                              grid_map::Position::Zero());
    in_gm.addObstacleLayerFromImage(img_src, 0.5);
    in_gm.updateDistanceLayer();
    in_gm.maps.setFrameId("/map");

    // Set publishers.
    ros::Publisher map_publisher =
        nh.advertise<nav_msgs::OccupancyGrid>("grid_map", 1, true);
    // Set subscribers.
    ros::Subscriber reference_sub =
        nh.subscribe("/clicked_point", 0, referenceCb);
    ros::Subscriber start_sub =
        nh.subscribe("/initialpose", 1, startCb);
    ros::Subscriber end_sub =
        nh.subscribe("/move_base_simple/goal", 1, goalCb);

    // Markers initialization.
    ros_viz_tools::RosVizTools markers(nh, "markers");
    std::string marker_frame_id = "/map";

    // Loop.
    ros::Rate rate(10.0);
    while (nh.ok()) {
        ros::Time time = ros::Time::now();
        markers.clear();
        int id = 0;

        // Cancel.
        if (reference_path.size() >= 2) {
            const auto &p1 = reference_path[reference_path.size() - 2];
            const auto &p2 = reference_path.back();
            if (hmpl::distance(p1, p2) <= 0.001) {
                reference_path.clear();
                reference_rcv = false;
            }
        }

        // Visualize reference path selected by mouse.
        visualization_msgs::Marker reference_marker =
            markers.newSphereList(0.5, "reference point", id++, ros_viz_tools::RED, marker_frame_id);
        for (size_t i = 0; i != reference_path.size(); ++i) {
            geometry_msgs::Point p;
            p.x = reference_path[i].x;
            p.y = reference_path[i].y;
            p.z = 1.0;
            reference_marker.points.push_back(p);
        }
        markers.append(reference_marker);
        // Visualize start and end point selected by mouse.
        geometry_msgs::Vector3 scale;
        scale.x = 2.0;
        scale.y = 0.3;
        scale.z = 0.3;
        geometry_msgs::Pose start_pose;
        start_pose.position.x = start_state.x;
        start_pose.position.y = start_state.y;
        start_pose.position.z = 1.0;
        auto start_quat = tf::createQuaternionFromYaw(start_state.z);
        start_pose.orientation.x = start_quat.x();
        start_pose.orientation.y = start_quat.y();
        start_pose.orientation.z = start_quat.z();
        start_pose.orientation.w = start_quat.w();
        visualization_msgs::Marker start_marker =
            markers.newArrow(scale, start_pose, "start point", id++, ros_viz_tools::CYAN, marker_frame_id);
        markers.append(start_marker);
        geometry_msgs::Pose end_pose;
        end_pose.position.x = end_state.x;
        end_pose.position.y = end_state.y;
        end_pose.position.z = 1.0;
        auto end_quat = tf::createQuaternionFromYaw(end_state.z);
        end_pose.orientation.x = end_quat.x();
        end_pose.orientation.y = end_quat.y();
        end_pose.orientation.z = end_quat.z();
        end_pose.orientation.w = end_quat.w();
        visualization_msgs::Marker end_marker =
            markers.newArrow(scale, end_pose, "end point", id++, ros_viz_tools::CYAN, marker_frame_id);
        markers.append(end_marker);

        // Calculate.
        std::vector<hmpl::State> result_path, smoothed_reference_path;
        if (reference_rcv && start_state_rcv && end_state_rcv) {
            PathOptimizationNS::PathOptimizer path_optimizer(reference_path, start_state, end_state, in_gm);
            if (path_optimizer.solve(&result_path)) {
                std::cout << "ok!" << std::endl;
            }
            smoothed_reference_path = path_optimizer.getSmoothedPath();
        }

        // Visualize result path.
        visualization_msgs::Marker result_marker =
            markers.newLineStrip(0.15, "optimized path", id++, ros_viz_tools::GREEN, marker_frame_id);
        for (size_t i = 0; i != result_path.size(); ++i) {
            geometry_msgs::Point p;
            p.x = result_path[i].x;
            p.y = result_path[i].y;
            p.z = 1.0;
            result_marker.points.push_back(p);
        }
        markers.append(result_marker);
        // Visualize smoothed reference path.
        visualization_msgs::Marker smoothed_reference_marker =
            markers.newLineStrip(0.07,
                                 "smoothed reference path",
                                 id++,
                                 ros_viz_tools::YELLOW,
                                 marker_frame_id);
        for (size_t i = 0; i != smoothed_reference_path.size(); ++i) {
            geometry_msgs::Point p;
            p.x = smoothed_reference_path[i].x;
            p.y = smoothed_reference_path[i].y;
            p.z = 1.0;
            smoothed_reference_marker.points.push_back(p);
        }
        markers.append(smoothed_reference_marker);
        visualization_msgs::Marker vehicle_geometry_marker =
            markers.newLineList(0.1, "vehicle", id++, ros_viz_tools::LIME_GREEN, marker_frame_id);
        // Visualize vehicle geometry.
        for (size_t i = 0; i != result_path.size(); ++i) {
            double heading = result_path[i].z;
            hmpl::State p1, p2, p3, p4;
            p1.x = 3.9;
            p1.y = 1;
            p2.x = 3.9;
            p2.y = -1;
            p3.x = -1.0;
            p3.y = -1;
            p4.x = -1.0;
            p4.y = 1;
            auto tmp_relto = result_path[i];
            tmp_relto.z = heading;
            p1 = hmpl::localToGlobal(tmp_relto, p1);
            p2 = hmpl::localToGlobal(tmp_relto, p2);
            p3 = hmpl::localToGlobal(tmp_relto, p3);
            p4 = hmpl::localToGlobal(tmp_relto, p4);
            geometry_msgs::Point pp1, pp2, pp3, pp4;
            pp1.x = p1.x;
            pp1.y = p1.y;
            pp1.z = 0.1;
            pp2.x = p2.x;
            pp2.y = p2.y;
            pp2.z = 0.1;
            pp3.x = p3.x;
            pp3.y = p3.y;
            pp3.z = 0.1;
            pp4.x = p4.x;
            pp4.y = p4.y;
            pp4.z = 0.1;
            vehicle_geometry_marker.points.push_back(pp1);
            vehicle_geometry_marker.points.push_back(pp2);
            vehicle_geometry_marker.points.push_back(pp2);
            vehicle_geometry_marker.points.push_back(pp3);
            vehicle_geometry_marker.points.push_back(pp3);
            vehicle_geometry_marker.points.push_back(pp4);
            vehicle_geometry_marker.points.push_back(pp4);
            vehicle_geometry_marker.points.push_back(pp1);
        }
        markers.append(vehicle_geometry_marker);

        // Publish the grid_map.
        in_gm.maps.setTimestamp(time.toNSec());
        nav_msgs::OccupancyGrid message;
        grid_map::GridMapRosConverter::toOccupancyGrid(
            in_gm.maps, in_gm.obs, in_gm.FREE, in_gm.OCCUPY, message);
        map_publisher.publish(message);

        // Publish markers.
        markers.publish();
        ROS_INFO("map published");

        // Wait for next cycle.
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}