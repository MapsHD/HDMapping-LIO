#pragma once

#include <vector>
#include <map>
#include <memory>
#include <mutex>
#include <Eigen/Dense>
#include <Fusion.h>
#include <structures.h>
#include <lidar_odometry_utils.h>

namespace hdmapping_lio {

class Pipeline {
public:
    Pipeline();

    LidarOdometryParams& params() { return params_; }

    void addImu(double timestamp, const FusionVector& gyro, const FusionVector& accel);

    bool addPointCloud(double timestamp, std::vector<Point3Di> points);

    Eigen::Affine3d getLatestPose() const;

    std::vector<Point3Di> getMapPoints() const;

private:
    void processChunk();

    LidarOdometryParams params_;
    FusionAhrs ahrs_;

    std::vector<Eigen::Affine3d> pose_buffer_;
    std::vector<std::pair<double, double>> timestamp_buffer_;
    std::vector<RawIMUData> imu_buffer_;
    std::vector<Eigen::Vector3d> imu_om_fi_ka_buffer_;

    std::vector<Point3Di> point_buffer_;
    std::vector<Point3Di> points_global_;

    LookupStats lookup_stats_;

    std::vector<Eigen::Affine3d> prev_trajectory_;
    std::vector<Eigen::Affine3d> prev_prev_trajectory_;

    Eigen::Affine3d latest_pose_ = Eigen::Affine3d::Identity();
    double prev_imu_timestamp_ = 0.0;
    bool first_imu_ = true;

    double acc_distance_ = 0.0;
    bool initialized_ = false;
    int initial_points_collected_ = 0;
    int chunk_index_ = -1;
};

}
