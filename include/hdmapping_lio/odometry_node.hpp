#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <hdmapping_lio/pipeline.hpp>

#include <mutex>
#include <queue>
#include <thread>
#include <atomic>
#include <condition_variable>

namespace hdmapping_lio {

class OdometryNode : public rclcpp::Node {
public:
    explicit OdometryNode(const rclcpp::NodeOptions& options);
    ~OdometryNode() override;

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void processingLoop();

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    nav_msgs::msg::Path path_msg_;

    std::unique_ptr<Pipeline> lio_;

    std::queue<sensor_msgs::msg::Imu::SharedPtr> imu_queue_;
    std::queue<sensor_msgs::msg::PointCloud2::SharedPtr> cloud_queue_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;

    std::thread processing_thread_;
    std::atomic<bool> running_{true};

    std::string odom_frame_;
    std::string base_frame_;
    double map_publish_rate_;
    rclcpp::Time last_map_publish_time_;
};

}
