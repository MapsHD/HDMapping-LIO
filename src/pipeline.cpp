#include <hdmapping_lio/pipeline.hpp>
#include <transformations.h>

namespace hdmapping_lio {

Pipeline::Pipeline()
{
    FusionAhrsInitialise(&ahrs_);
    ahrs_.settings.convention = FusionConventionNwu;
    ahrs_.settings.gain = static_cast<float>(params_.ahrs_gain);
}

void Pipeline::addImu(double timestamp, const FusionVector& gyro, const FusionVector& accel)
{
    if (first_imu_) {
        ahrs_.settings.gain = static_cast<float>(params_.ahrs_gain);
        FusionAhrsUpdateNoMagnetometer(&ahrs_, gyro, accel, 1.0f / 200.0f);
        first_imu_ = false;
    } else {
        float dt = static_cast<float>(timestamp - prev_imu_timestamp_);
        if (dt <= 0.0f) dt = 1.0f / 200.0f;
        FusionAhrsUpdateNoMagnetometer(&ahrs_, gyro, accel, dt);
    }
    prev_imu_timestamp_ = timestamp;

    FusionQuaternion quat = FusionAhrsGetQuaternion(&ahrs_);
    Eigen::Quaterniond q(quat.element.w, quat.element.x, quat.element.y, quat.element.z);
    Eigen::Affine3d pose = Eigen::Affine3d::Identity();
    pose.rotate(q);

    pose_buffer_.push_back(pose);
    timestamp_buffer_.emplace_back(timestamp, 0.0);

    RawIMUData raw;
    raw.timestamp = timestamp;
    raw.accelerometers = Eigen::Vector3d(accel.axis.x, accel.axis.y, accel.axis.z);
    raw.guroscopes = Eigen::Vector3d(gyro.axis.x, gyro.axis.y, gyro.axis.z);
    imu_buffer_.push_back(raw);

    TaitBryanPose tb = pose_tait_bryan_from_affine_matrix(pose);
    imu_om_fi_ka_buffer_.emplace_back(tb.om, tb.fi, tb.ka);
}

bool Pipeline::addPointCloud(double timestamp, std::vector<Point3Di> points)
{
    if (initial_points_collected_ <= params_.threshold_initial_points) {
        for (auto& p : points) {
            initial_points_collected_++;
            params_.initial_points.push_back(p);
            if (initial_points_collected_ > params_.threshold_initial_points)
                break;
        }
        if (initial_points_collected_ > params_.threshold_initial_points) {
            pose_buffer_.clear();
            timestamp_buffer_.clear();
            imu_buffer_.clear();
            imu_om_fi_ka_buffer_.clear();
            point_buffer_.clear();
        }
        return false;
    }

    point_buffer_.insert(point_buffer_.end(),
        std::make_move_iterator(points.begin()),
        std::make_move_iterator(points.end()));

    if (static_cast<int>(pose_buffer_.size()) >= params_.threshold_nr_poses) {
        processChunk();
        return true;
    }

    return false;
}

Eigen::Affine3d Pipeline::getLatestPose() const
{
    return latest_pose_;
}

std::vector<Point3Di> Pipeline::getMapPoints() const
{
    return points_global_;
}

void Pipeline::processChunk()
{
    const int threshold = params_.threshold_nr_poses;

    WorkerData wd;
    wd.intermediate_trajectory.reserve(threshold);
    wd.intermediate_trajectory_motion_model.reserve(threshold);
    wd.intermediate_trajectory_timestamps.reserve(threshold);
    wd.imu_om_fi_ka.reserve(threshold);
    wd.raw_imu_data.reserve(threshold);

    for (int i = 0; i < threshold; i++) {
        wd.intermediate_trajectory.push_back(pose_buffer_[i]);
        wd.intermediate_trajectory_motion_model.push_back(pose_buffer_[i]);
        wd.intermediate_trajectory_timestamps.push_back(timestamp_buffer_[i]);
        wd.imu_om_fi_ka.push_back(imu_om_fi_ka_buffer_[i]);
        wd.raw_imu_data.push_back(imu_buffer_[i]);
    }

    double ts_begin = wd.intermediate_trajectory_timestamps.front().first;
    double ts_end = wd.intermediate_trajectory_timestamps.back().first;

    std::vector<Point3Di> chunk_points;
    std::vector<Point3Di> remaining_points;

    for (auto& p : point_buffer_) {
        if (p.timestamp >= ts_begin && p.timestamp <= ts_end)
            chunk_points.push_back(std::move(p));
        else if (p.timestamp > ts_end)
            remaining_points.push_back(std::move(p));
    }

    pose_buffer_.erase(pose_buffer_.begin(), pose_buffer_.begin() + threshold);
    timestamp_buffer_.erase(timestamp_buffer_.begin(), timestamp_buffer_.begin() + threshold);
    imu_buffer_.erase(imu_buffer_.begin(), imu_buffer_.begin() + threshold);
    imu_om_fi_ka_buffer_.erase(imu_om_fi_ka_buffer_.begin(), imu_om_fi_ka_buffer_.begin() + threshold);
    point_buffer_ = std::move(remaining_points);

    if (wd.intermediate_trajectory_timestamps.size() > 2 && !chunk_points.empty()) {
        double ts_step = (ts_end - ts_begin) / chunk_points.size();
        for (size_t i = 0; i < chunk_points.size(); i++)
            chunk_points[i].timestamp = ts_begin + i * ts_step;
    }

    for (auto& p : chunk_points) {
        auto lower = std::lower_bound(
            wd.intermediate_trajectory_timestamps.begin(),
            wd.intermediate_trajectory_timestamps.end(),
            p.timestamp,
            [](const std::pair<double, double>& lhs, double rhs) {
                return lhs.first < rhs;
            });

        int index_pose = static_cast<int>(
            std::distance(wd.intermediate_trajectory_timestamps.begin(), lower)) - 1;

        if (index_pose >= 0 &&
            index_pose < static_cast<int>(wd.intermediate_trajectory_timestamps.size()) &&
            std::abs(p.timestamp - wd.intermediate_trajectory_timestamps[index_pose].first) <= 0.01)
            p.index_pose = index_pose;
        else
            p.index_pose = -1;
    }

    auto remove_it = std::remove_if(chunk_points.begin(), chunk_points.end(),
        [](const Point3Di& p) { return p.index_pose == -1; });
    chunk_points.erase(remove_it, chunk_points.end());

    if (chunk_points.size() <= 1000)
        return;

    std::vector<Point3Di> intermediate_points;
    if (params_.decimation > 0.0 && chunk_points.size() > 1000)
        intermediate_points = decimate(chunk_points, params_.decimation, params_.decimation, params_.decimation);
    else
        intermediate_points = std::move(chunk_points);

    if (!initialized_) {
        params_.m_g = wd.intermediate_trajectory[0];

        Eigen::Affine3d m_last = params_.m_g;
        auto tmp = wd.intermediate_trajectory;
        wd.intermediate_trajectory[0] = m_last;
        for (size_t k = 1; k < tmp.size(); k++) {
            Eigen::Affine3d m_update = tmp[k - 1].inverse() * tmp[k];
            m_last = m_last * m_update;
            wd.intermediate_trajectory[k] = m_last;
        }
        wd.intermediate_trajectory_motion_model = wd.intermediate_trajectory;

        auto pp = params_.initial_points;
        for (auto& p : pp)
            p.point = params_.m_g * p.point;

        {
            std::lock_guard<std::mutex> lock(params_.mutex_buckets_indoor);
            std::lock_guard<std::mutex> lock2(params_.mutex_buckets_outdoor);
            if (params_.ablation_study_use_hierarchical_rgd) {
                update_rgd_hierarchy(
                    params_.in_out_params_indoor, params_.buckets_indoor, pp,
                    params_.m_g.translation(),
                    params_.in_out_params_outdoor, params_.buckets_outdoor,
                    lookup_stats_);
            } else {
                update_rgd(params_.in_out_params_indoor, params_.buckets_indoor, pp,
                    params_.m_g.translation(), &lookup_stats_.indoor_lookups);
            }
        }

        initialized_ = true;
        chunk_index_ = 0;
    } else {
        chunk_index_++;

        Eigen::Affine3d m_last = prev_trajectory_.back();
        auto tmp = wd.intermediate_trajectory;
        wd.intermediate_trajectory[0] = m_last;
        for (size_t k = 1; k < tmp.size(); k++) {
            Eigen::Affine3d m_update = tmp[k - 1].inverse() * tmp[k];
            m_last = m_last * m_update;
            wd.intermediate_trajectory[k] = m_last;
        }

        if (chunk_index_ > 1 && params_.use_motion_from_previous_step &&
            !prev_prev_trajectory_.empty()) {
            Eigen::Vector3d mean_shift = prev_trajectory_.back().translation() -
                prev_prev_trajectory_.back().translation();
            mean_shift /= static_cast<double>(prev_trajectory_.size());

            if (mean_shift.norm() > 1.0)
                mean_shift = Eigen::Vector3d::Zero();

            for (size_t tr = 0; tr < wd.intermediate_trajectory.size(); tr++)
                wd.intermediate_trajectory[tr].translation() += mean_shift * static_cast<double>(tr);
        }

        wd.intermediate_trajectory_motion_model = wd.intermediate_trajectory;
    }

    auto tmp_trajectory = wd.intermediate_trajectory;
    double delta = 100000.0;
    double lm_factor = 1.0;

    auto t0 = std::chrono::steady_clock::now();

    for (int iter = 0; iter < params_.nr_iter; iter++) {
        std::lock_guard<std::mutex> lock(params_.mutex_buckets_indoor);
        std::lock_guard<std::mutex> lock2(params_.mutex_buckets_outdoor);

        delta = 100000.0;
        optimize_lidar_odometry(
            intermediate_points,
            wd.intermediate_trajectory,
            wd.intermediate_trajectory_motion_model,
            params_.in_out_params_indoor,
            params_.buckets_indoor,
            params_.in_out_params_outdoor,
            params_.buckets_outdoor,
            params_.useMultithread,
            params_.max_distance_lidar,
            delta,
            lm_factor,
            params_.motion_model_correction,
            params_.lidar_odometry_motion_model_x_1_sigma_m,
            params_.lidar_odometry_motion_model_y_1_sigma_m,
            params_.lidar_odometry_motion_model_z_1_sigma_m,
            params_.lidar_odometry_motion_model_om_1_sigma_deg,
            params_.lidar_odometry_motion_model_fi_1_sigma_deg,
            params_.lidar_odometry_motion_model_ka_1_sigma_deg,
            params_.lidar_odometry_motion_model_fix_origin_x_1_sigma_m,
            params_.lidar_odometry_motion_model_fix_origin_y_1_sigma_m,
            params_.lidar_odometry_motion_model_fix_origin_z_1_sigma_m,
            params_.lidar_odometry_motion_model_fix_origin_om_1_sigma_deg,
            params_.lidar_odometry_motion_model_fix_origin_fi_1_sigma_deg,
            params_.lidar_odometry_motion_model_fix_origin_ka_1_sigma_deg,
            params_.ablation_study_use_planarity,
            params_.ablation_study_use_norm,
            params_.ablation_study_use_hierarchical_rgd,
            params_.ablation_study_use_view_point_and_normal_vectors,
            lookup_stats_,
            params_.ablation_study_use_threshold_outer_rgd,
            delta,
            params_.convergence_delta_threshold_outer_rgd);

        if (delta < params_.convergence_delta_threshold)
            break;

        if (iter % 10 == 0 && iter > 0)
            lm_factor *= 10.0;

        if (params_.real_time_threshold_seconds > 0.0 &&
            std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() >
                params_.real_time_threshold_seconds)
            break;
    }

    double acc_distance_prev = acc_distance_;
    acc_distance_ += (wd.intermediate_trajectory.front().inverse() *
        wd.intermediate_trajectory.back()).translation().norm();

    if (!(acc_distance_ == acc_distance_)) {
        wd.intermediate_trajectory = tmp_trajectory;
        acc_distance_ = acc_distance_prev;
    } else {
        for (const auto& p : intermediate_points) {
            Point3Di pp = p;
            pp.point = wd.intermediate_trajectory[p.index_pose] * pp.point;
            points_global_.push_back(pp);
        }

        if (acc_distance_ > params_.sliding_window_trajectory_length_threshold) {
            if (params_.reference_points.empty()) {
                params_.buckets_indoor.clear();
                params_.buckets_outdoor.clear();
            }

            std::vector<Point3Di> points_global_new;
            points_global_new.reserve(points_global_.size() / 2 + 1);
            for (size_t k = points_global_.size() / 2; k < points_global_.size(); k++)
                points_global_new.emplace_back(points_global_[k]);

            acc_distance_ = 0.0;
            points_global_ = std::move(points_global_new);

            std::lock_guard<std::mutex> lock(params_.mutex_buckets_indoor);
            std::lock_guard<std::mutex> lock2(params_.mutex_buckets_outdoor);
            if (params_.ablation_study_use_hierarchical_rgd) {
                update_rgd_hierarchy(
                    params_.in_out_params_indoor, params_.buckets_indoor, points_global_,
                    wd.intermediate_trajectory[0].translation(),
                    params_.in_out_params_outdoor, params_.buckets_outdoor,
                    lookup_stats_);
            } else {
                update_rgd(params_.in_out_params_indoor, params_.buckets_indoor, points_global_,
                    wd.intermediate_trajectory[0].translation(), &lookup_stats_.indoor_lookups);
            }
        } else {
            std::vector<Point3Di> pg;
            pg.reserve(intermediate_points.size());
            for (const auto& p : intermediate_points) {
                Point3Di pp = p;
                pp.point = wd.intermediate_trajectory[p.index_pose] * pp.point;
                pg.push_back(pp);
            }

            std::lock_guard<std::mutex> lock(params_.mutex_buckets_indoor);
            std::lock_guard<std::mutex> lock2(params_.mutex_buckets_outdoor);
            if (params_.ablation_study_use_hierarchical_rgd) {
                update_rgd_hierarchy(
                    params_.in_out_params_indoor, params_.buckets_indoor, pg,
                    wd.intermediate_trajectory[0].translation(),
                    params_.in_out_params_outdoor, params_.buckets_outdoor,
                    lookup_stats_);
            } else {
                update_rgd(params_.in_out_params_indoor, params_.buckets_indoor, pg,
                    wd.intermediate_trajectory[0].translation(), &lookup_stats_.indoor_lookups);
            }
        }
    }

    latest_pose_ = wd.intermediate_trajectory.back();

    prev_prev_trajectory_ = std::move(prev_trajectory_);
    prev_trajectory_ = std::move(wd.intermediate_trajectory);
}

}
