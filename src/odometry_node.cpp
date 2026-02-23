#include <hdmapping_lio/odometry_node.hpp>
#include <hdmapping_lio/conversions.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <limits>

namespace hdmapping_lio {

OdometryNode::OdometryNode(const rclcpp::NodeOptions& options)
    : Node("hdmapping_lio_node", options)
{
    auto imu_topic = declare_parameter("imu_topic", "/imu");
    auto lidar_topic = declare_parameter("lidar_topic", "/points");
    odom_frame_ = declare_parameter("odom_frame", "odom");
    base_frame_ = declare_parameter("base_frame", "base_link");
    map_publish_rate_ = declare_parameter("map_publish_rate", 1.0);

    lio_ = std::make_unique<Pipeline>();
    auto& p = lio_->params();

    p.filter_threshold_xy_inner = declare_parameter("filter_threshold_xy_inner", 0.3);
    p.filter_threshold_xy_outer = declare_parameter("filter_threshold_xy_outer", 70.0);
    p.decimation = declare_parameter("decimation", 0.01);
    p.ahrs_gain = declare_parameter("ahrs_gain", 0.5);
    p.nr_iter = declare_parameter("nr_iter", 1000);
    p.real_time_threshold_seconds = declare_parameter("real_time_threshold_seconds", 10.0);
    p.threshold_nr_poses = declare_parameter("threshold_nr_poses", 20);

    double res_indoor = declare_parameter("ndt_resolution_indoor", 0.1);
    p.in_out_params_indoor.resolution_X = res_indoor;
    p.in_out_params_indoor.resolution_Y = res_indoor;
    p.in_out_params_indoor.resolution_Z = res_indoor;

    double res_outdoor = declare_parameter("ndt_resolution_outdoor", 0.3);
    p.in_out_params_outdoor.resolution_X = res_outdoor;
    p.in_out_params_outdoor.resolution_Y = res_outdoor;
    p.in_out_params_outdoor.resolution_Z = res_outdoor;

    p.sliding_window_trajectory_length_threshold =
        declare_parameter("sliding_window_trajectory_length_threshold", 200.0);
    p.max_distance_lidar = declare_parameter("max_distance_lidar", 70.0);

    double sigma_xyz = declare_parameter("motion_model_sigma_xyz", 0.0005);
    p.lidar_odometry_motion_model_x_1_sigma_m = sigma_xyz;
    p.lidar_odometry_motion_model_y_1_sigma_m = sigma_xyz;
    p.lidar_odometry_motion_model_z_1_sigma_m = sigma_xyz;

    double sigma_omfika = declare_parameter("motion_model_sigma_omfika", 0.01);
    p.lidar_odometry_motion_model_om_1_sigma_deg = sigma_omfika;
    p.lidar_odometry_motion_model_fi_1_sigma_deg = sigma_omfika;
    p.lidar_odometry_motion_model_ka_1_sigma_deg = sigma_omfika;

    RCLCPP_INFO(get_logger(),
        "NDT indoor=%.3f outdoor=%.3f nr_iter=%d time_limit=%.1fs threshold_nr_poses=%d decimation=%.4f sw=%.0fm",
        res_indoor, res_outdoor, p.nr_iter, p.real_time_threshold_seconds,
        p.threshold_nr_poses, p.decimation, p.sliding_window_trajectory_length_threshold);

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("hdmapping/odom", 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("hdmapping/trajectory", 10);
    map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("hdmapping/map", 10);

    path_msg_.header.frame_id = odom_frame_;

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, rclcpp::SensorDataQoS(),
        std::bind(&OdometryNode::imuCallback, this, std::placeholders::_1));

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        lidar_topic, rclcpp::SensorDataQoS(),
        std::bind(&OdometryNode::pointCloudCallback, this, std::placeholders::_1));

    last_map_publish_time_ = now();

    processing_thread_ = std::thread(&OdometryNode::processingLoop, this);

    RCLCPP_INFO(get_logger(), "Started. IMU: %s, LiDAR: %s", imu_topic.c_str(), lidar_topic.c_str());
}

OdometryNode::~OdometryNode()
{
    running_ = false;
    queue_cv_.notify_all();
    if (processing_thread_.joinable())
        processing_thread_.join();
}

void OdometryNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    if (!running_) return;
    std::lock_guard<std::mutex> lock(queue_mutex_);
    imu_queue_.push(msg);
    queue_cv_.notify_one();
}

void OdometryNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    if (!running_) return;
    std::lock_guard<std::mutex> lock(queue_mutex_);
    cloud_queue_.push(msg);
    queue_cv_.notify_one();
}

void OdometryNode::processingLoop()
{
    while (running_) {
        std::vector<sensor_msgs::msg::Imu::SharedPtr> imu_msgs;
        std::vector<sensor_msgs::msg::PointCloud2::SharedPtr> cloud_msgs;

        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            queue_cv_.wait(lock, [this] {
                return !running_ || !imu_queue_.empty() || !cloud_queue_.empty();
            });

            if (!running_) break;

            while (!imu_queue_.empty()) {
                imu_msgs.push_back(imu_queue_.front());
                imu_queue_.pop();
            }
            while (!cloud_queue_.empty()) {
                cloud_msgs.push_back(cloud_queue_.front());
                cloud_queue_.pop();
            }
        }

        size_t imu_idx = 0, cloud_idx = 0;

        while (running_ && (imu_idx < imu_msgs.size() || cloud_idx < cloud_msgs.size())) {
            double imu_ts = (imu_idx < imu_msgs.size())
                ? rclcpp::Time(imu_msgs[imu_idx]->header.stamp).seconds()
                : std::numeric_limits<double>::max();
            double cloud_ts = (cloud_idx < cloud_msgs.size())
                ? rclcpp::Time(cloud_msgs[cloud_idx]->header.stamp).seconds()
                : std::numeric_limits<double>::max();

            if (imu_ts <= cloud_ts) {
                auto [ts, gyro, accel] = imu_to_fusion_input(*imu_msgs[imu_idx]);
                lio_->addImu(ts, gyro, accel);
                imu_idx++;
            } else {
                const auto& msg = cloud_msgs[cloud_idx];
                auto points = pointcloud2_to_point3di(*msg,
                    lio_->params().filter_threshold_xy_inner,
                    lio_->params().filter_threshold_xy_outer);

                double ts = rclcpp::Time(msg->header.stamp).seconds();
                bool chunk_processed = lio_->addPointCloud(ts, std::move(points));

                if (chunk_processed) {
                    rclcpp::Time stamp = msg->header.stamp;
                    auto pose = lio_->getLatestPose();

                    auto odom_msg = affine_to_odometry(pose, stamp, odom_frame_, base_frame_);
                    odom_pub_->publish(odom_msg);
                    tf_broadcaster_->sendTransform(
                        affine_to_transform(pose, stamp, odom_frame_, base_frame_));

                    geometry_msgs::msg::PoseStamped ps;
                    ps.header = odom_msg.header;
                    ps.pose = odom_msg.pose.pose;
                    path_msg_.header.stamp = stamp;
                    path_msg_.poses.push_back(ps);
                    path_pub_->publish(path_msg_);

                    if (map_publish_rate_ > 0.0) {
                        double dt = (now() - last_map_publish_time_).seconds();
                        if (dt >= 1.0 / map_publish_rate_) {
                            auto map_points = lio_->getMapPoints();
                            map_pub_->publish(point3di_to_pointcloud2(map_points, stamp, odom_frame_));
                            last_map_publish_time_ = now();
                        }
                    }
                }
                cloud_idx++;
            }
        }
    }
}

}

RCLCPP_COMPONENTS_REGISTER_NODE(hdmapping_lio::OdometryNode)
