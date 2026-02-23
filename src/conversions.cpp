#include <hdmapping_lio/conversions.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

static constexpr float RAD_TO_DEG = 180.0f / static_cast<float>(M_PI);

namespace hdmapping_lio {

std::vector<Point3Di> pointcloud2_to_point3di(
    const sensor_msgs::msg::PointCloud2& msg,
    double filter_threshold_xy_inner,
    double filter_threshold_xy_outer)
{
    const double inner_sq = filter_threshold_xy_inner * filter_threshold_xy_inner;
    const double outer_sq = filter_threshold_xy_outer * filter_threshold_xy_outer;
    const double header_time = rclcpp::Time(msg.header.stamp).seconds();

    bool has_intensity = false;
    bool has_timestamp = false;
    std::string timestamp_field;
    uint8_t timestamp_datatype = 0;
    for (const auto& field : msg.fields) {
        if (field.name == "intensity") has_intensity = true;
        if (field.name == "timestamp" || field.name == "time" || field.name == "t") {
            has_timestamp = true;
            timestamp_field = field.name;
            timestamp_datatype = field.datatype;
        }
    }

    const size_t point_count = msg.width * msg.height;
    std::vector<Point3Di> points;
    points.reserve(point_count);

    sensor_msgs::PointCloud2ConstIterator<float> it_x(msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(msg, "z");

    std::optional<sensor_msgs::PointCloud2ConstIterator<float>> it_intensity;
    if (has_intensity)
        it_intensity.emplace(msg, "intensity");

    std::optional<sensor_msgs::PointCloud2ConstIterator<float>> it_time_f32;
    std::optional<sensor_msgs::PointCloud2ConstIterator<double>> it_time_f64;
    if (has_timestamp) {
        if (timestamp_datatype == sensor_msgs::msg::PointField::FLOAT64)
            it_time_f64.emplace(msg, timestamp_field);
        else
            it_time_f32.emplace(msg, timestamp_field);
    }

    for (size_t i = 0; i < point_count; ++i, ++it_x, ++it_y, ++it_z) {
        const double x = static_cast<double>(*it_x);
        const double y = static_cast<double>(*it_y);
        const double z = static_cast<double>(*it_z);

        const float intensity_val = has_intensity ? *(*it_intensity) : 0.0f;

        double point_time = header_time;
        if (has_timestamp) {
            double raw_time = it_time_f64 ? *(*it_time_f64)
                                          : static_cast<double>(*(*it_time_f32));
            if (raw_time > 1e15) {
                point_time = raw_time / 1e9;
            } else {
                point_time = header_time + raw_time;
            }
        }

        if (has_intensity) ++(*it_intensity);
        if (it_time_f64) ++(*it_time_f64);
        if (it_time_f32) ++(*it_time_f32);

        if (std::isnan(x) || std::isnan(y) || std::isnan(z)) continue;

        const double dist_sq = x * x + y * y;
        if (dist_sq < inner_sq || dist_sq > outer_sq) continue;

        Point3Di p;
        p.point = Eigen::Vector3d(x, y, z);
        p.intensity = intensity_val;
        p.timestamp = point_time;
        p.index_pose = -1;
        p.lidarid = 0;
        p.index_point = static_cast<int>(i);

        points.push_back(p);
    }

    return points;
}

std::tuple<double, FusionVector, FusionVector> imu_to_fusion_input(
    const sensor_msgs::msg::Imu& msg)
{
    double timestamp = rclcpp::Time(msg.header.stamp).seconds();

    FusionVector gyro = {
        static_cast<float>(msg.angular_velocity.x) * RAD_TO_DEG,
        static_cast<float>(msg.angular_velocity.y) * RAD_TO_DEG,
        static_cast<float>(msg.angular_velocity.z) * RAD_TO_DEG
    };

    FusionVector accel = {
        static_cast<float>(msg.linear_acceleration.x),
        static_cast<float>(msg.linear_acceleration.y),
        static_cast<float>(msg.linear_acceleration.z)
    };

    return {timestamp, gyro, accel};
}

nav_msgs::msg::Odometry affine_to_odometry(
    const Eigen::Affine3d& pose,
    const rclcpp::Time& stamp,
    const std::string& frame_id,
    const std::string& child_frame_id)
{
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = frame_id;
    odom.child_frame_id = child_frame_id;

    const Eigen::Vector3d t = pose.translation();
    odom.pose.pose.position.x = t.x();
    odom.pose.pose.position.y = t.y();
    odom.pose.pose.position.z = t.z();

    const Eigen::Quaterniond q(pose.rotation());
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    return odom;
}

geometry_msgs::msg::TransformStamped affine_to_transform(
    const Eigen::Affine3d& pose,
    const rclcpp::Time& stamp,
    const std::string& frame_id,
    const std::string& child_frame_id)
{
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = frame_id;
    tf.child_frame_id = child_frame_id;

    const Eigen::Vector3d t = pose.translation();
    tf.transform.translation.x = t.x();
    tf.transform.translation.y = t.y();
    tf.transform.translation.z = t.z();

    const Eigen::Quaterniond q(pose.rotation());
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();

    return tf;
}

sensor_msgs::msg::PointCloud2 point3di_to_pointcloud2(
    const std::vector<Point3Di>& points,
    const rclcpp::Time& stamp,
    const std::string& frame_id)
{
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;

    sensor_msgs::PointCloud2Modifier modifier(msg);
    modifier.setPointCloud2Fields(4,
        "x", 1, sensor_msgs::msg::PointField::FLOAT32,
        "y", 1, sensor_msgs::msg::PointField::FLOAT32,
        "z", 1, sensor_msgs::msg::PointField::FLOAT32,
        "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float> it_x(msg, "x");
    sensor_msgs::PointCloud2Iterator<float> it_y(msg, "y");
    sensor_msgs::PointCloud2Iterator<float> it_z(msg, "z");
    sensor_msgs::PointCloud2Iterator<float> it_i(msg, "intensity");

    for (const auto& p : points) {
        *it_x = static_cast<float>(p.point.x());
        *it_y = static_cast<float>(p.point.y());
        *it_z = static_cast<float>(p.point.z());
        *it_i = p.intensity;
        ++it_x; ++it_y; ++it_z; ++it_i;
    }

    return msg;
}

}
