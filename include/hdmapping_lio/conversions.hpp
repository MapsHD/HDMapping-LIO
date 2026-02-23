#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <Eigen/Dense>
#include <structures.h>
#include <Fusion.h>

namespace hdmapping_lio {

std::vector<Point3Di> pointcloud2_to_point3di(
    const sensor_msgs::msg::PointCloud2& msg,
    double filter_threshold_xy_inner,
    double filter_threshold_xy_outer);

std::tuple<double, FusionVector, FusionVector> imu_to_fusion_input(
    const sensor_msgs::msg::Imu& msg);

nav_msgs::msg::Odometry affine_to_odometry(
    const Eigen::Affine3d& pose,
    const rclcpp::Time& stamp,
    const std::string& frame_id,
    const std::string& child_frame_id);

geometry_msgs::msg::TransformStamped affine_to_transform(
    const Eigen::Affine3d& pose,
    const rclcpp::Time& stamp,
    const std::string& frame_id,
    const std::string& child_frame_id);

sensor_msgs::msg::PointCloud2 point3di_to_pointcloud2(
    const std::vector<Point3Di>& points,
    const rclcpp::Time& stamp,
    const std::string& frame_id);

}
