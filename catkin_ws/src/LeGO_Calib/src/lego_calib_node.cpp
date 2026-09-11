#include <Gril_Calib/Gril_Calib.h>
#include <Fusion/Fusion.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/Imu.h>
#include <ros/package.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <csignal>
#include <deque>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

// Optimization weights declared by the migrated GRIL-Calib core.
double GYRO_FACTOR_ = 10.0;
double ACC_FACTOR_ = 1.0;
double GROUND_FACTOR_ = 5.0;

namespace {
constexpr double kGravity = 9.81;
constexpr double kRadToDeg = 180.0 / M_PI;

struct TimedCloud {
    double stamp = 0.0;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
};
struct TimedOrientation {
    double stamp = 0.0;
    Eigen::Quaterniond q_ground_to_imu = Eigen::Quaterniond::Identity();
    bool valid = false;
};

class LeGOCalibNode {
public:
    LeGOCalibNode()
        : nh_(), pnh_("~"), calib_(new Gril_Calib()), jacobian_(MatrixXd::Zero(3 * 30000, 3)) {
        loadParameters();
        configureAhrs();
        result_path_ = resolvePath(result_path_);

        odom_sub_ = nh_.subscribe(odom_topic_, 100, &LeGOCalibNode::odomCallback, this);
        imu_sub_ = nh_.subscribe(imu_topic_, 1000, &LeGOCalibNode::imuCallback, this);
        cloud_sub_ = nh_.subscribe(cloud_topic_, 20, &LeGOCalibNode::cloudCallback, this);
        path_pub_ = nh_.advertise<nav_msgs::Path>("/lego_calib/lidar_path", 2, true);
        plane_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/lego_calib/ground_plane_inliers", 2, true);

        path_.header.frame_id = path_frame_id_;
        ROS_INFO_STREAM("LeGO_Calib is ready. odom=" << odom_topic_
                        << ", imu=" << imu_topic_ << ", cloud=" << cloud_topic_
                        << ". It consumes LeGO-LIO's mapOptimization output directly."
                        << " Fixed frontend T_IL used to recover LiDAR pose: t=["
                        << frontend_t_IL_.transpose() << "] m, RPY=["
                        << frontend_roll_ << ", " << frontend_pitch_ << ", "
                        << frontend_yaw_ << "] rad. Orientation source: "
                        << (use_imu_oriention_ ? "IMU message quaternion" : "Fusion AHRS") << ".");
    }

    ~LeGOCalibNode() { finalize("destructor"); }

    void finalize(const std::string& reason) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (finalized_) return;
        finalized_ = true;

        const std::size_t sample_count = accepted_samples_;
        if (sample_count < static_cast<std::size_t>(minimum_samples_to_solve_)) {
            ROS_WARN_STREAM("LeGO_Calib stopped (" << reason << ") with " << sample_count
                            << " accepted synchronized samples; need at least "
                            << minimum_samples_to_solve_
                            << ". No unreliable calibration was solved.");
            writeStatusFile(false, reason, "insufficient accepted samples");
            return;
        }

        ROS_WARN_STREAM("LeGO_Calib finalizing on " << reason << " with " << sample_count
                        << " samples. Running a batch calibration even if the excitation test is incomplete.");
        try {
            runCalibration(reason);
        } catch (const std::exception& error) {
            ROS_ERROR_STREAM("LeGO_Calib final calibration failed: " << error.what());
            writeStatusFile(false, reason, error.what());
        }
    }

private:
    void loadParameters() {
        pnh_.param<std::string>("topics/lidar_odom", odom_topic_, "/aft_mapped_to_init");
        pnh_.param<std::string>("topics/imu", imu_topic_, "/imu");
        pnh_.param<std::string>("topics/points", cloud_topic_, "/velodyne_points");
        // true: use sensor_msgs/Imu::orientation directly; false: use AHRS.
        pnh_.param<bool>("imu/use_imu_oriention", use_imu_oriention_, false);
        pnh_.param<std::string>("frames/path", path_frame_id_, "map");
        // Temporary de-embedding of the fixed LiDAR->IMU transform used by
        // LeGO-LIO.  Its odometry is the IMU/body trajectory because the
        // frontend transforms input points with this same transform.
        pnh_.param<double>("frontend_lidar_to_imu/roll", frontend_roll_, 0.0);
        pnh_.param<double>("frontend_lidar_to_imu/pitch", frontend_pitch_, 0.0);
        pnh_.param<double>("frontend_lidar_to_imu/yaw", frontend_yaw_, 0.0);
        pnh_.param<double>("frontend_lidar_to_imu/x", frontend_tx_, 0.0);
        pnh_.param<double>("frontend_lidar_to_imu/y", frontend_ty_, 0.0);
        pnh_.param<double>("frontend_lidar_to_imu/z", frontend_tz_, 0.0);
        frontend_R_IL_ =
            Eigen::AngleAxisd(frontend_yaw_, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(frontend_pitch_, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(frontend_roll_, Eigen::Vector3d::UnitX());
        frontend_t_IL_ = Eigen::Vector3d(frontend_tx_, frontend_ty_, frontend_tz_);
        pnh_.param<std::string>("output/result_file", result_path_, std::string(ROOT_DIR) + "result/lego_calib_result.yaml");

        pnh_.param<double>("sync/max_cloud_odom_dt", max_cloud_odom_dt_, 0.06);
        pnh_.param<double>("sync/max_imu_odom_dt", max_imu_odom_dt_, 0.03);
        pnh_.param<double>("runtime/movement_start_distance", movement_start_distance_, 0.05);
        pnh_.param<int>("runtime/minimum_samples_to_solve", minimum_samples_to_solve_, 8);
        pnh_.param<int>("runtime/max_samples", max_samples_, 30000);
        pnh_.param<double>("calibration/odom_frequency", odom_frequency_, 3.0);
        pnh_.param<int>("calibration/cut_frame_num", cut_frame_num_, 1);
        pnh_.param<double>("calibration/mean_acc_norm", mean_acc_norm_, kGravity);
        pnh_.param<double>("calibration/imu_sensor_height", calib_->imu_sensor_height, 0.1);
        pnh_.param<double>("calibration/data_accum_length", calib_->data_accum_length, 300.0);
        pnh_.param<double>("calibration/x_accumulate", calib_->x_accumulate, 0.01);
        pnh_.param<double>("calibration/y_accumulate", calib_->y_accumulate, 0.01);
        pnh_.param<double>("calibration/z_accumulate", calib_->z_accumulate, 0.99);
        pnh_.param<double>("calibration/trans_IL_x", calib_->trans_IL_x, 0.0);
        pnh_.param<double>("calibration/trans_IL_y", calib_->trans_IL_y, 0.0);
        pnh_.param<double>("calibration/trans_IL_z", calib_->trans_IL_z, 0.0);
        pnh_.param<double>("calibration/bound_th", calib_->bound_th, 0.3);
        pnh_.param<bool>("calibration/set_boundary", calib_->set_boundary, false);
        pnh_.param<bool>("calibration/verbose", calib_->verbose, false);
        pnh_.param<bool>("calibration/bspline_enable", calib_->bspline_enable, true);
        pnh_.param<int>("calibration/bspline_ctrl_points", calib_->bspline_ctrl_points, 0);
        pnh_.param<int>("calibration/bspline_degree", calib_->bspline_degree, 3);
        pnh_.param<double>("calibration/bspline_smoothing", calib_->bspline_smoothing, 1e-4);
        pnh_.param<double>("calibration/gyro_factor", GYRO_FACTOR_, 10.0);
        pnh_.param<double>("calibration/acc_factor", ACC_FACTOR_, 1.0);
        pnh_.param<double>("calibration/ground_factor", GROUND_FACTOR_, 5.0);

        pnh_.param<int>("ground_plane/max_iterations", plane_max_iterations_, 300);
        pnh_.param<int>("ground_plane/min_inliers", plane_min_inliers_, 500);
        pnh_.param<double>("ground_plane/min_inlier_ratio", plane_min_inlier_ratio_, 0.20);
        pnh_.param<double>("ground_plane/distance_threshold", plane_distance_threshold_, 0.03);
        pnh_.param<double>("ground_plane/max_rms", plane_max_rms_, 0.03);
        pnh_.param<double>("ground_plane/max_tilt_deg", plane_max_tilt_deg_, 25.0);
        pnh_.param<double>("ground_plane/voxel_leaf_size", plane_voxel_leaf_size_, 0.10);
        pnh_.param<double>("ground_plane/min_range", plane_min_range_, 1.0);
        pnh_.param<double>("ground_plane/max_range", plane_max_range_, 40.0);
        pnh_.param<double>("ground_plane/min_z", plane_min_z_, -3.0);
        pnh_.param<double>("ground_plane/max_z", plane_max_z_, 1.0);

        pnh_.param<double>("ahrs/nominal_frequency", ahrs_frequency_, 200.0);
        pnh_.param<double>("ahrs/gain", ahrs_gain_, 0.5);
        pnh_.param<double>("ahrs/acceleration_rejection", ahrs_acc_rejection_, 10.0);
        pnh_.param<double>("ahrs/max_dt", ahrs_max_dt_, 0.05);

        max_samples_ = std::max(max_samples_, minimum_samples_to_solve_ + 10);
        jacobian_.resize(3 * max_samples_, 3);
        jacobian_.setZero();
    }

    void configureAhrs() {
        FusionOffsetInitialise(&offset_, static_cast<unsigned int>(std::max(1.0, ahrs_frequency_)));
        FusionAhrsInitialise(&ahrs_);
        const FusionAhrsSettings settings = {
            static_cast<float>(ahrs_gain_), static_cast<float>(ahrs_acc_rejection_), 0.0f,
            static_cast<unsigned int>(std::max(1.0, 5.0 * ahrs_frequency_))};
        FusionAhrsSetSettings(&ahrs_, &settings);
        last_ahrs_stamp_ = 0.0;
    }

    static Eigen::Quaterniond quaternionFromMsg(const geometry_msgs::Quaternion& q) {
        return Eigen::Quaterniond(q.w, q.x, q.y, q.z);
    }

    std::string resolvePath(const std::string& path) const {
        if (!path.empty() && path.front() == '/') return path;
        const std::string package_path = ros::package::getPath("lego_calib");
        return package_path.empty() ? path : package_path + "/" + path;
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (finalized_) return;
        const double stamp = msg->header.stamp.toSec();
        if (stamp <= 0.0) return;
        if (last_imu_stamp_ > 0.0 && stamp <= last_imu_stamp_) {
            ROS_WARN("IMU timestamp reset detected; restarting LeGO_Calib accumulation.");
            resetAccumulation();
        }
        last_imu_stamp_ = stamp;
        calib_->push_ALL_IMU_CalibState(msg, mean_acc_norm_);

        if (use_imu_oriention_) {
            // Direct mode: use sensor_msgs/Imu::orientation without any
            // integration or accelerometer correction. The calibration core
            // expects the Ground -> IMU convention here.
            const auto& q_msg = msg->orientation;
            const bool orientation_available = msg->orientation_covariance[0] != -1.0;
            const bool quaternion_finite = std::isfinite(q_msg.x) && std::isfinite(q_msg.y) &&
                                           std::isfinite(q_msg.z) && std::isfinite(q_msg.w);
            const double quaternion_norm = std::sqrt(q_msg.x * q_msg.x + q_msg.y * q_msg.y +
                                                     q_msg.z * q_msg.z + q_msg.w * q_msg.w);
            if (orientation_available && quaternion_finite && quaternion_norm > 1e-6) {
                TimedOrientation orientation;
                orientation.stamp = stamp;
                orientation.q_ground_to_imu = Eigen::Quaterniond(q_msg.w, q_msg.x, q_msg.y, q_msg.z).normalized();
                orientation.valid = true;
                imu_orientations_.push_back(orientation);
                ROS_INFO_ONCE("LeGO_Calib is using the IMU-provided orientation quaternion directly.");
            } else {
                ROS_WARN_THROTTLE(5.0, "Ignoring IMU orientation at %.6f: unavailable or invalid (covariance[0]=%.1f, norm=%.6g).",
                                  stamp, msg->orientation_covariance[0], quaternion_norm);
            }
        } else {
            // AHRS mode: preserve the original Fusion-based attitude path.
            if (last_ahrs_stamp_ > 0.0) {
                const double dt = stamp - last_ahrs_stamp_;
                if (dt > 0.0 && dt <= ahrs_max_dt_) {
                    const auto& g = msg->angular_velocity;
                    const auto& a = msg->linear_acceleration;
                    FusionVector gyro = {static_cast<float>(g.x * kRadToDeg), static_cast<float>(g.y * kRadToDeg), static_cast<float>(g.z * kRadToDeg)};
                    FusionVector acc = {static_cast<float>(a.x / kGravity), static_cast<float>(a.y / kGravity), static_cast<float>(a.z / kGravity)};
                    gyro = FusionOffsetUpdate(&offset_, gyro);
                    FusionAhrsUpdateNoMagnetometer(&ahrs_, gyro, acc, static_cast<float>(dt));
                    const FusionQuaternion fq = FusionAhrsGetQuaternion(&ahrs_);
                    TimedOrientation orientation;
                    orientation.stamp = stamp;
                    orientation.q_ground_to_imu = Eigen::Quaterniond(fq.element.w, fq.element.x, fq.element.y, fq.element.z).normalized();
                    orientation.valid = !FusionAhrsGetFlags(&ahrs_).initialising;
                    if (orientation.valid) {
                        imu_orientations_.push_back(orientation);
                    }
                }
            }
            last_ahrs_stamp_ = stamp;
            ROS_INFO_ONCE("LeGO_Calib is using the Fusion AHRS orientation path.");
        }
        trimQueues(stamp);
    }

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (finalized_) return;
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*msg, *cloud);
        TimedCloud timed;
        timed.stamp = msg->header.stamp.toSec();

        // RoboSense stores absolute per-point timestamps.  The raw ROS
        // header is the end of the scan, while LeGO-LIO publishes odometry
        // at the scan start after Rb32Handler normalizes the header.  Use the
        // minimum point timestamp here, otherwise this callback pairs an
        // odometry sample with the previous scan's ground cloud (the end of
        // scan k is approximately the start of scan k+1).
        for (const sensor_msgs::PointField& field : msg->fields) {
            if (field.name != "timestamp") continue;
            try {
                double point_time_min = std::numeric_limits<double>::infinity();
                const std::size_t point_count =
                    static_cast<std::size_t>(msg->width) * msg->height;
                if (field.datatype == sensor_msgs::PointField::FLOAT64) {
                    sensor_msgs::PointCloud2ConstIterator<double> it(*msg, "timestamp");
                    for (std::size_t i = 0; i < point_count; ++i, ++it)
                        point_time_min = std::min(point_time_min, *it);
                } else if (field.datatype == sensor_msgs::PointField::FLOAT32) {
                    sensor_msgs::PointCloud2ConstIterator<float> it(*msg, "timestamp");
                    for (std::size_t i = 0; i < point_count; ++i, ++it)
                        point_time_min = std::min(point_time_min, static_cast<double>(*it));
                }
                if (std::isfinite(point_time_min)) timed.stamp = point_time_min;
            } catch (const std::exception&) {
                ROS_WARN_THROTTLE(5.0, "Failed to read raw cloud point timestamps; using header time.");
            }
            break;
        }
        timed.cloud = cloud;
        if (timed.stamp > 0.0) cloud_queue_.push_back(timed);
        ++cloud_count_;
        ROS_INFO_THROTTLE(5.0, "Received raw clouds=%zu latest_stamp=%.6f points=%zu queue=%zu", cloud_count_, timed.stamp, cloud->size(), cloud_queue_.size());
        trimQueues(timed.stamp);
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (finalized_ || solved_) return;
        const double stamp = msg->header.stamp.toSec();
        if (stamp <= 0.0) return;
        if (last_odom_stamp_ > 0.0 && stamp <= last_odom_stamp_) {
            ROS_WARN("LeGO-LIO odometry timestamp reset detected; restarting LeGO_Calib accumulation.");
            resetAccumulation();
        }
        last_odom_stamp_ = stamp;
        ++odom_count_;
        const TimedCloud* timed_cloud = closestCloud(stamp);
        ROS_INFO_THROTTLE(5.0, "Received mapped odom=%zu stamp=%.6f closest_cloud_dt=%.4f cloud_queue=%zu imu_orientations=%zu", odom_count_, stamp, timed_cloud ? std::abs(timed_cloud->stamp - stamp) : -1.0, cloud_queue_.size(), imu_orientations_.size());
        const TimedOrientation* imu_orientation = closestOrientation(stamp);
        if (!timed_cloud || !imu_orientation || !imu_orientation->valid) {
            ROS_WARN_THROTTLE(2.0, "Waiting for synchronized raw cloud and valid IMU-provided orientation (cloud=%d orient=%d valid=%d).", timed_cloud != nullptr, imu_orientation != nullptr, imu_orientation ? imu_orientation->valid : 0);
            return;
        }
        if (std::abs(timed_cloud->stamp - stamp) > max_cloud_odom_dt_ ||
            std::abs(imu_orientation->stamp - stamp) > max_imu_odom_dt_) {
            ROS_WARN_THROTTLE(2.0, "Skipping odom: cloud_dt=%.4f imu_dt=%.4f tolerance=(%.4f,%.4f).", std::abs(timed_cloud->stamp - stamp), std::abs(imu_orientation->stamp - stamp), max_cloud_odom_dt_, max_imu_odom_dt_);
            return;
        }

        // /aft_mapped_to_init is expressed for the IMU/body trajectory,
        // because LeGO-LIO first applies p_imu = R_IL p_lidar + t_IL to
        // every input point.  Recover the LiDAR-origin pose before passing it
        // to GRIL-Calib: T_WL = T_WI * T_IL.
        Eigen::Quaterniond q_map_imu = quaternionFromMsg(msg->pose.pose.orientation);
        if (q_map_imu.norm() < 1e-6) return;
        q_map_imu.normalize();
        const Eigen::Vector3d p_map_imu(msg->pose.pose.position.x,
                                        msg->pose.pose.position.y,
                                        msg->pose.pose.position.z);
        const Eigen::Quaterniond q_map_lidar =
            (q_map_imu * Eigen::Quaterniond(frontend_R_IL_)).normalized();
        const V3D position = p_map_imu + q_map_imu * frontend_t_IL_;
        if (!started_) {
            initial_position_ = position;
            started_ = true;
        }
        if (!moving_ && (position - initial_position_).norm() >= movement_start_distance_) {
            moving_ = true;
            move_start_time_ = stamp;
            ROS_INFO("Motion detected. LeGO_Calib now accumulates synchronized calibration samples.");
        }
        if (!moving_) return;

        Eigen::Quaterniond q_ground_lidar;
        V3D normal_lidar;
        double lidar_height = 0.0;
        pcl::PointCloud<pcl::PointXYZI>::Ptr inliers(new pcl::PointCloud<pcl::PointXYZI>());
        if (!fitGroundPlane(*timed_cloud->cloud, q_ground_lidar, normal_lidar, lidar_height, inliers)) {
            ++plane_reject_count_;
            ROS_WARN_THROTTLE(2.0, "Ground plane rejected for odom stamp %.6f (reject_count=%zu).", stamp, plane_reject_count_);
            return;
        }

        V3D lidar_omega = Zero3d;
        if (have_previous_pose_) {
            const double dt = stamp - previous_pose_stamp_;
            const M3D relative_rotation = previous_pose_rotation_.transpose() * q_map_lidar.toRotationMatrix();
            if (dt > 1e-4) lidar_omega = Log(relative_rotation) / dt;
        }
        previous_pose_rotation_ = q_map_lidar.toRotationMatrix();
        previous_pose_stamp_ = stamp;
        have_previous_pose_ = true;
        calib_->push_Lidar_CalibState(q_map_lidar.toRotationMatrix(), position, lidar_omega, Zero3d, stamp);
        calib_->push_Plane_Constraint(q_ground_lidar, imu_orientation->q_ground_to_imu, normal_lidar, lidar_height);
        publishPath(msg->header, msg->pose.pose);
        publishPlane(*inliers, msg->header);

        int frame = static_cast<int>(accepted_samples_);
        ++accepted_samples_;
        if (frame >= max_samples_) {
            ROS_WARN("Configured maximum sample count reached; finalizing calibration.");
            runCalibration("maximum sample count");
            return;
        }
        QD imu_q_for_assessment = imu_orientation->q_ground_to_imu;
        bool sufficiently_excited = calib_->data_sufficiency_assess(jacobian_, frame, lidar_omega,
                                                                     odom_frequency_, cut_frame_num_,
                                                                     q_ground_lidar, imu_q_for_assessment,
                                                                     lidar_height);
        if (sufficiently_excited && accepted_samples_ >= static_cast<std::size_t>(minimum_samples_to_solve_)) {
            runCalibration("automatic excitation criterion");
        }
    }

    bool fitGroundPlane(const pcl::PointCloud<pcl::PointXYZI>& input, Eigen::Quaterniond& q_ground_lidar,
                        V3D& normal_out, double& height_out,
                        pcl::PointCloud<pcl::PointXYZI>::Ptr& inliers_out) const {
        pcl::PointCloud<pcl::PointXYZI>::Ptr candidates(new pcl::PointCloud<pcl::PointXYZI>());
        candidates->reserve(input.size());
        for (const auto& p : input.points) {
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
            const double range = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
            if (range >= plane_min_range_ && range <= plane_max_range_ && p.z >= plane_min_z_ && p.z <= plane_max_z_)
                candidates->push_back(p);
        }
        if (candidates->size() < static_cast<std::size_t>(plane_min_inliers_)) {
            ROS_WARN_THROTTLE(2.0, "Ground candidate reject: candidates=%zu < min_inliers=%d (input=%zu).", candidates->size(), plane_min_inliers_, input.size());
            return false;
        }
        pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::VoxelGrid<pcl::PointXYZI> voxel;
        voxel.setLeafSize(plane_voxel_leaf_size_, plane_voxel_leaf_size_, plane_voxel_leaf_size_);
        voxel.setInputCloud(candidates);
        voxel.filter(*downsampled);
        if (downsampled->size() < static_cast<std::size_t>(plane_min_inliers_)) {
            ROS_WARN_THROTTLE(2.0, "Ground voxel reject: downsampled=%zu < min_inliers=%d (candidate=%zu).", downsampled->size(), plane_min_inliers_, candidates->size());
            return false;
        }

        pcl::SACSegmentation<pcl::PointXYZI> ransac;
        ransac.setOptimizeCoefficients(true);
        ransac.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
        ransac.setMethodType(pcl::SAC_RANSAC);
        ransac.setMaxIterations(plane_max_iterations_);
        ransac.setDistanceThreshold(plane_distance_threshold_);
        ransac.setAxis(Eigen::Vector3f::UnitZ());
        ransac.setEpsAngle(static_cast<float>(plane_max_tilt_deg_ * M_PI / 180.0));
        ransac.setInputCloud(downsampled);
        pcl::PointIndices indices;
        pcl::ModelCoefficients coefficients;
        ransac.segment(indices, coefficients);
        if (coefficients.values.size() < 4 || indices.indices.size() < static_cast<std::size_t>(plane_min_inliers_)) {
            ROS_WARN_THROTTLE(2.0, "Ground RANSAC reject: coefficients=%zu inliers=%zu/%zu.", coefficients.values.size(), indices.indices.size(), downsampled->size());
            return false;
        }

        Eigen::Vector3d normal(coefficients.values[0], coefficients.values[1], coefficients.values[2]);
        const double norm = normal.norm();
        if (norm < 1e-9) return false;
        normal /= norm;
        double d = coefficients.values[3] / norm;
        if (normal.z() < 0.0) { normal = -normal; d = -d; }
        const double tilt = std::acos(std::max(-1.0, std::min(1.0, normal.z()))) * kRadToDeg;
        const double ratio = static_cast<double>(indices.indices.size()) / static_cast<double>(downsampled->size());
        double squared_error = 0.0;
        for (const int index : indices.indices) {
            const auto& p = downsampled->points[index];
            const double residual = normal.dot(Eigen::Vector3d(p.x, p.y, p.z)) + d;
            squared_error += residual * residual;
            inliers_out->push_back(p);
        }
        const double rms = std::sqrt(squared_error / std::max<std::size_t>(1, indices.indices.size()));
        if (ratio < plane_min_inlier_ratio_ || rms > plane_max_rms_ || tilt > plane_max_tilt_deg_) {
            ROS_WARN_THROTTLE(2.0, "Ground quality reject: ratio=%.3f rms=%.3f tilt=%.2f (limits %.3f %.3f %.2f), inliers=%zu/%zu.", ratio, rms, tilt, plane_min_inlier_ratio_, plane_max_rms_, plane_max_tilt_deg_, indices.indices.size(), downsampled->size());
            return false;
        }
        normal_out = normal;
        height_out = std::abs(d);
        q_ground_lidar = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), normal).normalized();
        ROS_INFO_THROTTLE(2.0, "Ground plane accepted: inliers=%zu/%zu, height=%.3f m, tilt=%.2f deg, rms=%.3f m.",
                          indices.indices.size(), downsampled->size(), height_out, tilt, rms);
        return true;
    }

    const TimedCloud* closestCloud(double stamp) const {
        if (cloud_queue_.empty()) return nullptr;
        const TimedCloud* best = nullptr; double best_dt = std::numeric_limits<double>::infinity();
        for (const auto& item : cloud_queue_) { const double dt = std::abs(item.stamp - stamp); if (dt < best_dt) { best_dt = dt; best = &item; } }
        return best;
    }
    const TimedOrientation* closestOrientation(double stamp) const {
        if (imu_orientations_.empty()) return nullptr;
        const TimedOrientation* best = nullptr; double best_dt = std::numeric_limits<double>::infinity();
        for (const auto& item : imu_orientations_) { const double dt = std::abs(item.stamp - stamp); if (dt < best_dt) { best_dt = dt; best = &item; } }
        return best;
    }
    void trimQueues(double newest_stamp) {
        const double oldest = newest_stamp - 5.0;
        while (!cloud_queue_.empty() && cloud_queue_.front().stamp < oldest) cloud_queue_.pop_front();
        while (!imu_orientations_.empty() && imu_orientations_.front().stamp < oldest) imu_orientations_.pop_front();
    }
    void publishPath(const std_msgs::Header& header, const geometry_msgs::Pose& pose) {
        geometry_msgs::PoseStamped point; point.header = header; point.header.frame_id = path_frame_id_; point.pose = pose;
        path_.header = point.header; path_.poses.push_back(point); path_pub_.publish(path_);
    }
    void publishPlane(const pcl::PointCloud<pcl::PointXYZI>& cloud, const std_msgs::Header& header) {
        sensor_msgs::PointCloud2 msg; pcl::toROSMsg(cloud, msg); msg.header = header; plane_pub_.publish(msg);
    }
    void runCalibration(const std::string& reason) {
        if (solved_) return;
        if (accepted_samples_ < static_cast<std::size_t>(minimum_samples_to_solve_)) return;
        solved_ = true;
        try {
            double odom_freq = odom_frequency_;
            int cut = cut_frame_num_;
            double time_offset = 0.0;
            calib_->LI_Calibration(odom_freq, cut, time_offset, move_start_time_);
            writeResultFile(reason, true, "batch optimization completed");
            ROS_INFO_STREAM("LeGO_Calib result written to " << result_path_);
        } catch (...) { solved_ = false; throw; }
    }
    void writeStatusFile(bool solved, const std::string& reason, const std::string& status) const {
        std::ofstream output(result_path_);
        output << "status: " << (solved ? "solved" : "not_solved") << "\nreason: \"" << reason << "\"\nmessage: \"" << status << "\"\naccepted_samples: " << accepted_samples_ << "\n";
    }
    void writeResultFile(const std::string& reason, bool solved, const std::string& status) const {
        std::ofstream output(result_path_);
        if (!output) { ROS_ERROR_STREAM("Cannot write result file: " << result_path_); return; }
        const M3D R_LI = calib_->get_R_LI();
        const V3D t_LI = calib_->get_T_LI();
        const Eigen::Quaterniond q_LI(R_LI);
        output << std::setprecision(15);
        output << "status: " << (solved ? "solved" : "not_solved") << "\nreason: \"" << reason << "\"\nmessage: \"" << status << "\"\n";
        output << "accepted_samples: " << accepted_samples_ << "\n";
        output << "transform_convention: p_imu = R_lidar_to_imu * p_lidar + t_lidar_to_imu\n";
        output << "time_lag_imu_to_lidar_s: " << calib_->get_time_result() << "\n";
        output << "rotation_matrix_lidar_to_imu:\n";
        for (int r = 0; r < 3; ++r) output << "  - [" << R_LI(r,0) << ", " << R_LI(r,1) << ", " << R_LI(r,2) << "]\n";
        output << "quaternion_lidar_to_imu_xyzw: [" << q_LI.x() << ", " << q_LI.y() << ", " << q_LI.z() << ", " << q_LI.w() << "]\n";
        output << "translation_lidar_to_imu_m: [" << t_LI.x() << ", " << t_LI.y() << ", " << t_LI.z() << "]\n";
        const V3D euler = RotMtoEuler(R_LI) * kRadToDeg;
        output << "rpy_lidar_to_imu_deg: [" << euler.x() << ", " << euler.y() << ", " << euler.z() << "]\n";
        const V3D bg = calib_->get_gyro_bias(), ba = calib_->get_acc_bias();
        output << "gyro_bias_rad_s: [" << bg.x() << ", " << bg.y() << ", " << bg.z() << "]\n";
        output << "accel_bias_m_s2: [" << ba.x() << ", " << ba.y() << ", " << ba.z() << "]\n";
    }
    void resetAccumulation() {
        calib_.reset(new Gril_Calib());
        loadParameters();
        configureAhrs();
        accepted_samples_ = 0; started_ = false; moving_ = false; solved_ = false;
        have_previous_pose_ = false; previous_pose_stamp_ = 0.0;
        cloud_queue_.clear(); imu_orientations_.clear(); path_.poses.clear();
    }

    ros::NodeHandle nh_, pnh_;
    ros::Subscriber odom_sub_, imu_sub_, cloud_sub_;
    ros::Publisher path_pub_, plane_pub_;
    std::unique_ptr<Gril_Calib> calib_;
    std::mutex mutex_;
    std::deque<TimedCloud> cloud_queue_;
    std::deque<TimedOrientation> imu_orientations_;
    nav_msgs::Path path_;
    MatrixXd jacobian_;
    FusionAhrs ahrs_{};
    FusionOffset offset_{};
    std::string odom_topic_, imu_topic_, cloud_topic_, path_frame_id_, result_path_;
    double max_cloud_odom_dt_{}, max_imu_odom_dt_{}, movement_start_distance_{}, mean_acc_norm_{}, ahrs_frequency_{}, ahrs_gain_{}, ahrs_acc_rejection_{}, ahrs_max_dt_{};
    double frontend_roll_{}, frontend_pitch_{}, frontend_yaw_{}, frontend_tx_{}, frontend_ty_{}, frontend_tz_{};
    Eigen::Matrix3d frontend_R_IL_{Eigen::Matrix3d::Identity()};
    Eigen::Vector3d frontend_t_IL_{Eigen::Vector3d::Zero()};
    int minimum_samples_to_solve_{}, max_samples_{}, cut_frame_num_{}, plane_max_iterations_{}, plane_min_inliers_{};
    double odom_frequency_{};
    double plane_min_inlier_ratio_{}, plane_distance_threshold_{}, plane_max_rms_{}, plane_max_tilt_deg_{}, plane_voxel_leaf_size_{}, plane_min_range_{}, plane_max_range_{}, plane_min_z_{}, plane_max_z_{};
    double last_imu_stamp_ = 0.0, last_ahrs_stamp_ = 0.0, last_odom_stamp_ = 0.0, move_start_time_ = 0.0;
    V3D initial_position_ = Zero3d;
    M3D previous_pose_rotation_ = Eye3d;
    double previous_pose_stamp_ = 0.0;
    bool have_previous_pose_ = false;
    std::size_t accepted_samples_ = 0, cloud_count_ = 0, odom_count_ = 0, plane_reject_count_ = 0;
    bool use_imu_oriention_ = false;
    bool started_ = false, moving_ = false, solved_ = false, finalized_ = false;
};
} // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "lego_calib");
    LeGOCalibNode node;
    ros::spin();
    node.finalize("ROS shutdown (including Ctrl+C)");
    return 0;
}
