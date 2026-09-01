// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
//
// LeGO-LOAM scan-to-map mapping frontend, adapted for standard ROS REP-103
// FLU coordinates (x forward, y left, z up).
//
// This file intentionally contains only:
//   1. local-map scan-to-map registration, and
//   2. the odometry factors used by GTSAM/iSAM2.
// Loop-closure, historical ICP, GPS and global-map visualization code are not
// part of this node.

#include "utility.h"

#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/PointCloud2.h>

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

using gtsam::BetweenFactor;
using gtsam::ISAM2;
using gtsam::ISAM2Params;
using gtsam::NonlinearFactorGraph;
using gtsam::Point3;
using gtsam::Pose3;
using gtsam::PriorFactor;
using gtsam::Rot3;
using gtsam::Values;

namespace {
constexpr float kMaxFeatureCorrespondenceSqDist = 1.0f;
constexpr int kFeatureNeighbourCount = 5;
constexpr float kKeyFrameDistance = 0.3f;

struct PoseState {
    Eigen::Matrix3f rotation = Eigen::Matrix3f::Identity();
    Eigen::Vector3f translation = Eigen::Vector3f::Zero();
};

struct Scan2MapStats {
    bool mapReady = false;
    int iterations = 0;
    size_t correspondences = 0;
};

Eigen::Matrix3f rotationFromRPY(float roll, float pitch, float yaw)
{
    // ROS fixed-axis RPY: R = Rz(yaw) * Ry(pitch) * Rx(roll).
    return (Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY())
        * Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()))
        .toRotationMatrix();
}

void rpyFromRotation(const Eigen::Matrix3f& rotation, float& roll, float& pitch, float& yaw)
{
    const float sinPitch = std::max(-1.0f, std::min(1.0f, -rotation(2, 0)));
    pitch = std::asin(sinPitch);
    if (std::abs(std::cos(pitch)) > 1e-6f) {
        roll = std::atan2(rotation(2, 1), rotation(2, 2));
        yaw = std::atan2(rotation(1, 0), rotation(0, 0));
    } else {
        roll = 0.0f;
        yaw = std::atan2(-rotation(0, 1), rotation(1, 1));
    }
}

PoseState stateFromArray(const float pose[6])
{
    PoseState state;
    state.rotation = rotationFromRPY(pose[0], pose[1], pose[2]);
    state.translation = Eigen::Vector3f(pose[3], pose[4], pose[5]);
    return state;
}

void stateToArray(const PoseState& state, float pose[6])
{
    rpyFromRotation(state.rotation, pose[0], pose[1], pose[2]);
    pose[3] = state.translation.x();
    pose[4] = state.translation.y();
    pose[5] = state.translation.z();
}

PoseState compose(const PoseState& lhs, const PoseState& rhs)
{
    PoseState result;
    result.rotation = lhs.rotation * rhs.rotation;
    result.translation = lhs.rotation * rhs.translation + lhs.translation;
    return result;
}

PoseState inverse(const PoseState& state)
{
    PoseState result;
    result.rotation = state.rotation.transpose();
    result.translation = -result.rotation * state.translation;
    return result;
}

Pose3 gtsamPose(const PoseState& state)
{
    float roll, pitch, yaw;
    rpyFromRotation(state.rotation, roll, pitch, yaw);
    // GTSAM's RzRyRx(x, y, z) constructor takes the angles in x/y/z
    // order; for REP-103 that is (roll, pitch, yaw).  Passing (yaw, pitch,
    // roll) would put vehicle yaw into the X rotation and make the registered
    // cloud rotate around X.
    return Pose3(
        Rot3::RzRyRx(roll, pitch, yaw), Point3(state.translation.x(), state.translation.y(), state.translation.z()));
}

PoseState stateFromGtsamPose(const Pose3& pose)
{
    PoseState state;
    state.rotation = pose.rotation().matrix().cast<float>();
    state.translation = pose.translation().cast<float>();
    return state;
}

pcl::PointCloud<PointType>::Ptr transformCloud(const pcl::PointCloud<PointType>::Ptr& input, const PoseState& pose)
{
    pcl::PointCloud<PointType>::Ptr output(new pcl::PointCloud<PointType>());
    output->resize(input->size());
    for (size_t i = 0; i < input->size(); ++i) {
        const PointType& source = input->points[i];
        const Eigen::Vector3f transformed
            = pose.rotation * Eigen::Vector3f(source.x, source.y, source.z) + pose.translation;
        PointType& target = output->points[i];
        target = source;
        target.x = transformed.x();
        target.y = transformed.y();
        target.z = transformed.z();
    }
    return output;
}

} // namespace

class MapOptimization {
private:
    ros::NodeHandle nh_;

    ros::Publisher pubLaserCloudSurround_;
    ros::Publisher pubOdomAftMapped_;
    ros::Publisher pubKeyPoses_;
    ros::Publisher pubRegisteredCloud_;

    ros::Subscriber subLaserCloudCornerLast_;
    ros::Subscriber subLaserCloudSurfLast_;
    ros::Subscriber subOutlierCloudLast_;
    ros::Subscriber subLaserOdometry_;

    pcl::PointCloud<PointType>::Ptr laserCloudCornerLast_;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLast_;
    pcl::PointCloud<PointType>::Ptr laserCloudOutlierLast_;
    pcl::PointCloud<PointType>::Ptr laserCloudCornerLastDS_;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfTotalLastDS_;

    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap_;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap_;
    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMapDS_;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMapDS_;

    pcl::PointCloud<PointType>::Ptr laserCloudOri_;
    pcl::PointCloud<PointType>::Ptr coeffSel_;

    pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D_;
    pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D_;
    std::vector<pcl::PointCloud<PointType>::Ptr> cornerCloudKeyFrames_;
    std::vector<pcl::PointCloud<PointType>::Ptr> surfCloudKeyFrames_;
    std::vector<pcl::PointCloud<PointType>::Ptr> outlierCloudKeyFrames_;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap_;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap_;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurroundingKeyPoses_;
    pcl::VoxelGrid<PointType> downSizeFilterCorner_;
    pcl::VoxelGrid<PointType> downSizeFilterSurf_;
    pcl::VoxelGrid<PointType> downSizeFilterOutlier_;

    double timeLaserCloudCornerLast_ = 0.0;
    double timeLaserCloudSurfLast_ = 0.0;
    double timeLaserCloudOutlierLast_ = 0.0;
    double timeLaserOdometry_ = 0.0;
    double timeLastProcessing_ = -1.0;
    double mappingProcessInterval_ = mappingProcessInterval;
    double surroundingKeyframeSearchRadius_ = surroundingKeyframeSearchRadius;

    bool newLaserCloudCornerLast_ = false;
    bool newLaserCloudSurfLast_ = false;
    bool newLaserCloudOutlierLast_ = false;
    bool newLaserOdometry_ = false;

    float transformSum_[6] = { 0, 0, 0, 0, 0, 0 }; // latest frontend odometry pose
    float transformTobeMapped_[6] = { 0, 0, 0, 0, 0, 0 }; // latest map pose

    bool hasLastProcessedPose_ = false;
    PoseState lastOdomPose_;
    PoseState lastMappedPose_;
    PoseState lastKeyframeMappedPose_;
    bool hasLastKeyframeMappedPose_ = false;
    PointType previousRobotPosPoint_ { };

    std::vector<int> pointSearchInd_;
    std::vector<float> pointSearchSqDis_;
    std::vector<int> surroundingKeyPoseIds_;

    NonlinearFactorGraph gtSAMgraph_;
    Values initialEstimate_;
    Values isamCurrentEstimate_;
    std::unique_ptr<ISAM2> isam_;
    gtsam::noiseModel::Diagonal::shared_ptr priorNoise_;
    gtsam::noiseModel::Diagonal::shared_ptr odometryNoise_;

    float mappingCornerLeafSize_ = 0.2f;
    float mappingSurfLeafSize_ = 0.4f;
    float mappingOutlierLeafSize_ = 0.4f;

public:
    MapOptimization()
        : nh_("~")
    {
        nh_.param("mappingProcessInterval", mappingProcessInterval_, mappingProcessInterval_);
        nh_.param(
            "surroundingKeyframeSearchRadius", surroundingKeyframeSearchRadius_, surroundingKeyframeSearchRadius_);
        nh_.param("mappingCornerLeafSize", mappingCornerLeafSize_, mappingCornerLeafSize_);
        nh_.param("mappingSurfLeafSize", mappingSurfLeafSize_, mappingSurfLeafSize_);
        nh_.param("mappingOutlierLeafSize", mappingOutlierLeafSize_, mappingOutlierLeafSize_);

        pubKeyPoses_ = nh_.advertise<sensor_msgs::PointCloud2>("/key_pose_origin", 2);
        pubLaserCloudSurround_ = nh_.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surround", 2);
        pubOdomAftMapped_ = nh_.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 5);
        pubRegisteredCloud_ = nh_.advertise<sensor_msgs::PointCloud2>("/registered_cloud", 2);

        subLaserCloudCornerLast_ = nh_.subscribe<sensor_msgs::PointCloud2>(
            "/laser_cloud_corner_last", 2, &MapOptimization::laserCloudCornerLastHandler, this);
        subLaserCloudSurfLast_ = nh_.subscribe<sensor_msgs::PointCloud2>(
            "/laser_cloud_surf_last", 2, &MapOptimization::laserCloudSurfLastHandler, this);
        subOutlierCloudLast_ = nh_.subscribe<sensor_msgs::PointCloud2>(
            "/outlier_cloud_last", 2, &MapOptimization::laserCloudOutlierLastHandler, this);
        subLaserOdometry_
            = nh_.subscribe<nav_msgs::Odometry>("/laser_odom_to_init", 5, &MapOptimization::laserOdometryHandler, this);

        downSizeFilterCorner_.setLeafSize(mappingCornerLeafSize_, mappingCornerLeafSize_, mappingCornerLeafSize_);
        downSizeFilterSurf_.setLeafSize(mappingSurfLeafSize_, mappingSurfLeafSize_, mappingSurfLeafSize_);
        downSizeFilterOutlier_.setLeafSize(mappingOutlierLeafSize_, mappingOutlierLeafSize_, mappingOutlierLeafSize_);

        allocateMemory();
        initializeGtsam();

        ROS_INFO("MapOptimization started: local scan-to-map + GTSAM odometry "
                 "factors; loop closure disabled.");
    }

    ~MapOptimization() = default;

    void run()
    {
        if (!newLaserCloudCornerLast_ || !newLaserCloudSurfLast_ || !newLaserCloudOutlierLast_ || !newLaserOdometry_)
            return;

        const bool synchronized = std::abs(timeLaserCloudCornerLast_ - timeLaserOdometry_) < 0.01
            && std::abs(timeLaserCloudSurfLast_ - timeLaserOdometry_) < 0.01
            && std::abs(timeLaserCloudOutlierLast_ - timeLaserOdometry_) < 0.01;
        if (!synchronized)
            return;

        newLaserCloudCornerLast_ = false;
        newLaserCloudSurfLast_ = false;
        newLaserCloudOutlierLast_ = false;
        newLaserOdometry_ = false;

        if (timeLaserOdometry_ - timeLastProcessing_ < mappingProcessInterval_)
            return;
        timeLastProcessing_ = timeLaserOdometry_;

        initializeMapGuessFromOdometry();
        extractSurroundingKeyFrames();
        downsampleCurrentScan();

        // Keep the odometry-propagated initial guess so the log below reports
        // only the correction introduced by scan-to-map, before GTSAM runs.
        const PoseState scan2MapInitialGuess = currentMappedPose();
        const Scan2MapStats scan2MapStats = scan2MapOptimization();
        logScan2MapCorrection(scan2MapInitialGuess, currentMappedPose(), scan2MapStats);

        updateMappedPose();
        saveKeyFrameAndOdometryFactor();
        publishOutputs();
        clearLocalMap();
    }

private:
    void allocateMemory()
    {
        laserCloudCornerLast_.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfLast_.reset(new pcl::PointCloud<PointType>());
        laserCloudOutlierLast_.reset(new pcl::PointCloud<PointType>());
        laserCloudCornerLastDS_.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfTotalLastDS_.reset(new pcl::PointCloud<PointType>());

        laserCloudCornerFromMap_.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfFromMap_.reset(new pcl::PointCloud<PointType>());
        laserCloudCornerFromMapDS_.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfFromMapDS_.reset(new pcl::PointCloud<PointType>());
        laserCloudOri_.reset(new pcl::PointCloud<PointType>());
        coeffSel_.reset(new pcl::PointCloud<PointType>());

        cloudKeyPoses3D_.reset(new pcl::PointCloud<PointType>());
        cloudKeyPoses6D_.reset(new pcl::PointCloud<PointTypePose>());

        kdtreeCornerFromMap_.reset(new pcl::KdTreeFLANN<PointType>());
        kdtreeSurfFromMap_.reset(new pcl::KdTreeFLANN<PointType>());
        kdtreeSurroundingKeyPoses_.reset(new pcl::KdTreeFLANN<PointType>());
    }

    void initializeGtsam()
    {
        ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.01;
        parameters.relinearizeSkip = 1;
        isam_.reset(new ISAM2(parameters));

        gtsam::Vector priorVariances(6);
        priorVariances << 1e-6, 1e-6, 1e-6, 1e-8, 1e-8, 1e-6;
        priorNoise_ = gtsam::noiseModel::Diagonal::Variances(priorVariances);

        gtsam::Vector odometryVariances(6);
        odometryVariances << 1e-6, 1e-6, 1e-6, 1e-8, 1e-8, 1e-6;
        odometryNoise_ = gtsam::noiseModel::Diagonal::Variances(odometryVariances);
    }

    void laserCloudCornerLastHandler(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        timeLaserCloudCornerLast_ = msg->header.stamp.toSec();
        laserCloudCornerLast_->clear();
        pcl::fromROSMsg(*msg, *laserCloudCornerLast_);
        newLaserCloudCornerLast_ = true;
    }

    void laserCloudSurfLastHandler(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        timeLaserCloudSurfLast_ = msg->header.stamp.toSec();
        laserCloudSurfLast_->clear();
        pcl::fromROSMsg(*msg, *laserCloudSurfLast_);
        newLaserCloudSurfLast_ = true;
    }

    void laserCloudOutlierLastHandler(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        timeLaserCloudOutlierLast_ = msg->header.stamp.toSec();
        laserCloudOutlierLast_->clear();
        pcl::fromROSMsg(*msg, *laserCloudOutlierLast_);
        newLaserCloudOutlierLast_ = true;
    }

    void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr& msg)
    {
        timeLaserOdometry_ = msg->header.stamp.toSec();

        tf::Quaternion quaternion;
        tf::quaternionMsgToTF(msg->pose.pose.orientation, quaternion);
        double roll, pitch, yaw;
        tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

        // The frontend already publishes standard ROS FLU odometry. Do not
        // apply the camera/lidar axis permutation used by LeGO-LOAM.
        transformSum_[0] = static_cast<float>(roll);
        transformSum_[1] = static_cast<float>(pitch);
        transformSum_[2] = static_cast<float>(yaw);
        transformSum_[3] = static_cast<float>(msg->pose.pose.position.x);
        transformSum_[4] = static_cast<float>(msg->pose.pose.position.y);
        transformSum_[5] = static_cast<float>(msg->pose.pose.position.z);
        newLaserOdometry_ = true;
    }

    void initializeMapGuessFromOdometry()
    {
        const PoseState currentOdom = stateFromArray(transformSum_);
        PoseState guess = currentOdom;
        if (hasLastProcessedPose_) {
            // M_t = M_{t-1} * (O_{t-1}^{-1} * O_t). This carries the last
            // map correction forward while using the frontend odometry as the
            // initial guess for the current scan-to-map optimization.
            guess = compose(lastMappedPose_, compose(inverse(lastOdomPose_), currentOdom));
        }
        stateToArray(guess, transformTobeMapped_);
    }

    PoseState currentMappedPose() const { return stateFromArray(transformTobeMapped_); }

    void extractSurroundingKeyFrames()
    {
        laserCloudCornerFromMap_->clear();
        laserCloudSurfFromMap_->clear();
        surroundingKeyPoseIds_.clear();

        if (cloudKeyPoses3D_->empty())
            return;

        const PoseState guess = currentMappedPose();
        PointType searchPoint;
        searchPoint.x = guess.translation.x();
        searchPoint.y = guess.translation.y();
        searchPoint.z = guess.translation.z();
        searchPoint.intensity = 0.0f;

        kdtreeSurroundingKeyPoses_->setInputCloud(cloudKeyPoses3D_);
        pointSearchInd_.clear();
        pointSearchSqDis_.clear();
        kdtreeSurroundingKeyPoses_->radiusSearch(
            searchPoint, surroundingKeyframeSearchRadius_, pointSearchInd_, pointSearchSqDis_, 0);

        // Use the exact ids returned by the radius search. Averaging the
        // intensity field through PCL VoxelGrid would also average pose ids,
        // which is not a valid keyframe index.
        for (const int id : pointSearchInd_) {
            if (id >= 0 && id < static_cast<int>(cloudKeyPoses6D_->size())
                && std::find(surroundingKeyPoseIds_.begin(), surroundingKeyPoseIds_.end(), id)
                    == surroundingKeyPoseIds_.end())
                surroundingKeyPoseIds_.push_back(id);
        }

        for (const int id : surroundingKeyPoseIds_) {
            const PoseState pose = poseFromKeyFrame(id);
            *laserCloudCornerFromMap_ += *transformCloud(cornerCloudKeyFrames_[id], pose);
            *laserCloudSurfFromMap_ += *transformCloud(surfCloudKeyFrames_[id], pose);
            *laserCloudSurfFromMap_ += *transformCloud(outlierCloudKeyFrames_[id], pose);
        }

        if (!laserCloudCornerFromMap_->empty()) {
            downSizeFilterCorner_.setInputCloud(laserCloudCornerFromMap_);
            downSizeFilterCorner_.filter(*laserCloudCornerFromMapDS_);
        }
        if (!laserCloudSurfFromMap_->empty()) {
            downSizeFilterSurf_.setInputCloud(laserCloudSurfFromMap_);
            downSizeFilterSurf_.filter(*laserCloudSurfFromMapDS_);
        }
    }

    PoseState poseFromKeyFrame(int id) const
    {
        float pose[6]
            = { cloudKeyPoses6D_->points[id].roll, cloudKeyPoses6D_->points[id].pitch, cloudKeyPoses6D_->points[id].yaw,
                  cloudKeyPoses6D_->points[id].x, cloudKeyPoses6D_->points[id].y, cloudKeyPoses6D_->points[id].z };
        return stateFromArray(pose);
    }

    void downsampleCurrentScan()
    {
        laserCloudCornerLastDS_->clear();
        if (!laserCloudCornerLast_->empty()) {
            downSizeFilterCorner_.setInputCloud(laserCloudCornerLast_);
            downSizeFilterCorner_.filter(*laserCloudCornerLastDS_);
        }

        pcl::PointCloud<PointType>::Ptr surfTotal(new pcl::PointCloud<PointType>());
        *surfTotal += *laserCloudSurfLast_;
        *surfTotal += *laserCloudOutlierLast_;
        laserCloudSurfTotalLastDS_->clear();
        if (!surfTotal->empty()) {
            downSizeFilterSurf_.setInputCloud(surfTotal);
            downSizeFilterSurf_.filter(*laserCloudSurfTotalLastDS_);
        }
    }

    PointType associatePointToMap(const PointType& point) const
    {
        const PoseState pose = currentMappedPose();
        const Eigen::Vector3f p = pose.rotation * Eigen::Vector3f(point.x, point.y, point.z) + pose.translation;
        PointType result = point;
        result.x = p.x();
        result.y = p.y();
        result.z = p.z();
        return result;
    }

    void cornerOptimization()
    {
        if (laserCloudCornerFromMapDS_->size() < kFeatureNeighbourCount)
            return;
        kdtreeCornerFromMap_->setInputCloud(laserCloudCornerFromMapDS_);

        for (const PointType& pointOri : laserCloudCornerLastDS_->points) {
            const PointType pointSel = associatePointToMap(pointOri);
            pointSearchInd_.clear();
            pointSearchSqDis_.clear();
            if (kdtreeCornerFromMap_->nearestKSearch(
                    pointSel, kFeatureNeighbourCount, pointSearchInd_, pointSearchSqDis_)
                < kFeatureNeighbourCount)
                continue;
            if (pointSearchSqDis_[4] >= kMaxFeatureCorrespondenceSqDist)
                continue;

            Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
            for (const int id : pointSearchInd_) {
                const PointType& point = laserCloudCornerFromMapDS_->points[id];
                centroid += Eigen::Vector3f(point.x, point.y, point.z);
            }
            centroid /= static_cast<float>(kFeatureNeighbourCount);

            Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
            for (const int id : pointSearchInd_) {
                const PointType& point = laserCloudCornerFromMapDS_->points[id];
                const Eigen::Vector3f delta = Eigen::Vector3f(point.x, point.y, point.z) - centroid;
                covariance += delta * delta.transpose();
            }
            covariance /= static_cast<float>(kFeatureNeighbourCount);
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);
            if (solver.info() != Eigen::Success || solver.eigenvalues()(2) < 3.0f * solver.eigenvalues()(1))
                continue;

            const Eigen::Vector3f direction = solver.eigenvectors().col(2).normalized();
            const Eigen::Vector3f offset = Eigen::Vector3f(pointSel.x, pointSel.y, pointSel.z) - centroid;
            const Eigen::Vector3f perpendicular = offset - direction * offset.dot(direction);
            const float distance = perpendicular.norm();
            if (distance < 1e-5f)
                continue;

            const Eigen::Vector3f normal = perpendicular / distance;
            const float weight = 1.0f - 0.9f * std::abs(distance);
            if (weight <= 0.1f)
                continue;

            PointType coeff;
            coeff.x = weight * normal.x();
            coeff.y = weight * normal.y();
            coeff.z = weight * normal.z();
            // Signed point-to-line distance.  The previous implementation only
            // stored -n*c and therefore used a residual independent of pointSel;
            // that drives LM away from the map.
            coeff.intensity
                = weight * (normal.dot(Eigen::Vector3f(pointSel.x, pointSel.y, pointSel.z)) - normal.dot(centroid));
            laserCloudOri_->push_back(pointOri);
            coeffSel_->push_back(coeff);
        }
    }

    void surfOptimization()
    {
        if (laserCloudSurfFromMapDS_->size() < kFeatureNeighbourCount)
            return;
        kdtreeSurfFromMap_->setInputCloud(laserCloudSurfFromMapDS_);

        for (const PointType& pointOri : laserCloudSurfTotalLastDS_->points) {
            const PointType pointSel = associatePointToMap(pointOri);
            pointSearchInd_.clear();
            pointSearchSqDis_.clear();
            if (kdtreeSurfFromMap_->nearestKSearch(pointSel, kFeatureNeighbourCount, pointSearchInd_, pointSearchSqDis_)
                < kFeatureNeighbourCount)
                continue;
            if (pointSearchSqDis_[4] >= kMaxFeatureCorrespondenceSqDist)
                continue;

            Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
            for (const int id : pointSearchInd_) {
                const PointType& point = laserCloudSurfFromMapDS_->points[id];
                centroid += Eigen::Vector3f(point.x, point.y, point.z);
            }
            centroid /= static_cast<float>(kFeatureNeighbourCount);

            Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
            for (const int id : pointSearchInd_) {
                const PointType& point = laserCloudSurfFromMapDS_->points[id];
                const Eigen::Vector3f delta = Eigen::Vector3f(point.x, point.y, point.z) - centroid;
                covariance += delta * delta.transpose();
            }
            covariance /= static_cast<float>(kFeatureNeighbourCount);
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);
            if (solver.info() != Eigen::Success)
                continue;

            const Eigen::Vector3f normal = solver.eigenvectors().col(0).normalized();
            const float planeOffset = -normal.dot(centroid);
            bool planeValid = true;
            for (const int id : pointSearchInd_) {
                const PointType& point = laserCloudSurfFromMapDS_->points[id];
                const float residual = normal.x() * point.x + normal.y() * point.y + normal.z() * point.z + planeOffset;
                if (std::abs(residual) > 0.2f) {
                    planeValid = false;
                    break;
                }
            }
            if (!planeValid)
                continue;

            const Eigen::Vector3f selected(pointSel.x, pointSel.y, pointSel.z);
            const float residual = normal.dot(selected) + planeOffset;
            const float rangeFactor = std::sqrt(std::max(1.0f, selected.squaredNorm()));
            const float weight = 1.0f - 0.9f * std::abs(residual) / rangeFactor;
            if (weight <= 0.1f)
                continue;

            PointType coeff;
            coeff.x = weight * normal.x();
            coeff.y = weight * normal.y();
            coeff.z = weight * normal.z();
            coeff.intensity = weight * (normal.dot(selected) + planeOffset);
            laserCloudOri_->push_back(pointOri);
            coeffSel_->push_back(coeff);
        }
    }

    void residualJacobian(const PointType& point, const PointType& coeff, Eigen::Matrix<float, 1, 6>& jacobian) const
    {
        const float roll = transformTobeMapped_[0];
        const float pitch = transformTobeMapped_[1];
        const float yaw = transformTobeMapped_[2];
        const float cr = std::cos(roll), sr = std::sin(roll);
        const float cp = std::cos(pitch), sp = std::sin(pitch);
        const float cy = std::cos(yaw), sy = std::sin(yaw);
        Eigen::Matrix3f Rx, Ry, Rz, dRx, dRy, dRz;
        Rx << 1, 0, 0, 0, cr, -sr, 0, sr, cr;
        Ry << cp, 0, sp, 0, 1, 0, -sp, 0, cp;
        Rz << cy, -sy, 0, sy, cy, 0, 0, 0, 1;
        dRx << 0, 0, 0, 0, -sr, -cr, 0, cr, -sr;
        dRy << -sp, 0, cp, 0, 0, 0, -cp, 0, -sp;
        dRz << -sy, -cy, 0, cy, -sy, 0, 0, 0, 0;
        const Eigen::Vector3f p(point.x, point.y, point.z);
        const Eigen::Vector3f dRoll = Rz * Ry * dRx * p;
        const Eigen::Vector3f dPitch = Rz * dRy * Rx * p;
        const Eigen::Vector3f dYaw = dRz * Ry * Rx * p;
        const Eigen::Vector3f normal(coeff.x, coeff.y, coeff.z);
        jacobian(0, 0) = normal.dot(dRoll);
        jacobian(0, 1) = normal.dot(dPitch);
        jacobian(0, 2) = normal.dot(dYaw);
        jacobian(0, 3) = coeff.x;
        jacobian(0, 4) = coeff.y;
        jacobian(0, 5) = coeff.z;
    }

    bool optimizePose()
    {
        const int count = static_cast<int>(laserCloudOri_->size());
        if (count < 20)
            return false;

        Eigen::MatrixXf A(count, 6);
        Eigen::VectorXf b(count);
        for (int i = 0; i < count; ++i) {
            Eigen::Matrix<float, 1, 6> jacobian;
            residualJacobian(laserCloudOri_->points[i], coeffSel_->points[i], jacobian);
            A.row(i) = jacobian;
            b(i) = -coeffSel_->points[i].intensity;
        }

        // Same least-squares update as LeGO-LOAM's LMOptimization.  The
        // correspondence construction above follows its line and plane residuals.
        Eigen::Matrix<float, 6, 1> delta = A.colPivHouseholderQr().solve(b);
        if (!delta.allFinite())
            return false;

        for (int i = 0; i < 6; ++i)
            transformTobeMapped_[i] += delta(i);

        const float deltaR = std::sqrt(std::pow(delta(0) * 180.0f / static_cast<float>(M_PI), 2)
            + std::pow(delta(1) * 180.0f / static_cast<float>(M_PI), 2)
            + std::pow(delta(2) * 180.0f / static_cast<float>(M_PI), 2));
        const float deltaT = 100.0f * delta.tail<3>().norm();
        return deltaR < 0.05f && deltaT < 0.05f;
    }

    Scan2MapStats scan2MapOptimization()
    {
        Scan2MapStats stats;
        if (laserCloudCornerFromMapDS_->size() <= 10 || laserCloudSurfFromMapDS_->size() <= 100)
            return stats;

        stats.mapReady = true;

        // This is deliberately the original LeGO-LOAM mapping loop:
        // correspondence search -> corner/surface residuals -> LM update,
        // repeated for at most ten iterations.
        kdtreeCornerFromMap_->setInputCloud(laserCloudCornerFromMapDS_);
        kdtreeSurfFromMap_->setInputCloud(laserCloudSurfFromMapDS_);
        for (int iter = 0; iter < 10; ++iter) {
            laserCloudOri_->clear();
            coeffSel_->clear();
            cornerOptimization();
            surfOptimization();
            stats.iterations = iter + 1;
            stats.correspondences = laserCloudOri_->size();
            if (optimizePose())
                break;
        }
        return stats;
    }

    void logScan2MapCorrection(
        const PoseState& initialGuess, const PoseState& optimizedPose, const Scan2MapStats& stats) const
    {
        if (!stats.mapReady)
            return;

        // T_after = T_before * deltaScan, so deltaScan is expressed in the
        // FLU body frame of the odometry-propagated initial guess.
        const PoseState deltaScan = compose(inverse(initialGuess), optimizedPose);
        float scanRoll, scanPitch, scanYaw;
        rpyFromRotation(deltaScan.rotation, scanRoll, scanPitch, scanYaw);

        // Also report the accumulated mapping correction relative to the raw
        // frontend laser odometry at this timestamp. This includes corrections
        // carried forward from previous mapping frames.
        const PoseState laserOdomPose = stateFromArray(transformSum_);
        const PoseState deltaFromLaserOdom = compose(inverse(laserOdomPose), optimizedPose);
        float totalRoll, totalPitch, totalYaw;
        rpyFromRotation(deltaFromLaserOdom.rotation, totalRoll, totalPitch, totalYaw);

        constexpr float kRadToDeg = 180.0f / static_cast<float>(M_PI);
        const float scanRotationMagnitude
            = Eigen::AngleAxisf(deltaScan.rotation).angle() * kRadToDeg;
        const float totalRotationMagnitude
            = Eigen::AngleAxisf(deltaFromLaserOdom.rotation).angle() * kRadToDeg;

        // The project logger writes this record to ~log/file, configured in
        // params.yaml. The {node} token is expanded to the ROS node name.
        LOGF_INFO("[scan2map correction] stamp=%.6f, iterations=%d, correspondences=%zu\n"
                  "  scan-only (initial guess -> optimized, local FLU): "
                  "dxyz=[%+.4f, %+.4f, %+.4f] m, drpy=[%+.3f, %+.3f, %+.3f] deg, "
                  "|dt|=%.4f m, |dR|=%.3f deg\n"
                  "  total (laser odometry -> mapped, local FLU):       "
                  "dxyz=[%+.4f, %+.4f, %+.4f] m, drpy=[%+.3f, %+.3f, %+.3f] deg, "
                  "|dt|=%.4f m, |dR|=%.3f deg",
            timeLaserOdometry_, stats.iterations, stats.correspondences,
            deltaScan.translation.x(), deltaScan.translation.y(), deltaScan.translation.z(),
            scanRoll * kRadToDeg, scanPitch * kRadToDeg, scanYaw * kRadToDeg,
            deltaScan.translation.norm(), scanRotationMagnitude,
            deltaFromLaserOdom.translation.x(), deltaFromLaserOdom.translation.y(),
            deltaFromLaserOdom.translation.z(), totalRoll * kRadToDeg, totalPitch * kRadToDeg,
            totalYaw * kRadToDeg, deltaFromLaserOdom.translation.norm(), totalRotationMagnitude);

        // INFO is normally buffered by the logger. Explicitly flush the
        // low-rate correction record to preserve it during bag replay.
        lego_lio::log::Logger::instance().flush();
    }

    void updateMappedPose()
    {
        lastMappedPose_ = currentMappedPose();
        lastOdomPose_ = stateFromArray(transformSum_);
        hasLastProcessedPose_ = true;
    }

    void saveKeyFrameAndOdometryFactor()
    {
        const PoseState mappedPose = currentMappedPose();
        const PointType& previous = previousRobotPosPoint_;
        const Eigen::Vector3f currentPosition = mappedPose.translation;
        const float moved = (currentPosition - Eigen::Vector3f(previous.x, previous.y, previous.z)).norm();
        if (!cloudKeyPoses3D_->empty() && moved < kKeyFrameDistance)
            return;

        // The GTSAM key index is the keyframe index; the factor below references
        // the previous mapped keyframe pose.
        const size_t key = cloudKeyPoses3D_->size();
        if (key > 0 && !hasLastKeyframeMappedPose_)
            return;

        previousRobotPosPoint_.x = currentPosition.x();
        previousRobotPosPoint_.y = currentPosition.y();
        previousRobotPosPoint_.z = currentPosition.z();

        if (key == 0) {
            gtSAMgraph_.add(PriorFactor<Pose3>(0, gtsamPose(mappedPose), priorNoise_));
            initialEstimate_.insert(0, gtsamPose(mappedPose));
        } else {
            // This is the same odometry factor used by LeGO-LOAM's mapping
            // backend: the relative motion between two consecutive *mapped*
            // keyframes.  Using the raw frontend absolute-odometry increment here
            // makes iSAM2 erase the scan-to-map correction at every keyframe,
            // which appears as a jump in /aft_mapped_to_init.
            const Pose3 odometryMeasurement = gtsamPose(compose(inverse(lastKeyframeMappedPose_), mappedPose));
            gtSAMgraph_.add(BetweenFactor<Pose3>(key - 1, key, odometryMeasurement, odometryNoise_));
            initialEstimate_.insert(key, gtsamPose(mappedPose));
        }

        isam_->update(gtSAMgraph_, initialEstimate_);
        isam_->update();
        gtSAMgraph_.resize(0);
        initialEstimate_.clear();
        isamCurrentEstimate_ = isam_->calculateEstimate();

        const PoseState optimizedPose = stateFromGtsamPose(isamCurrentEstimate_.at<Pose3>(key));
        stateToArray(optimizedPose, transformTobeMapped_);
        lastMappedPose_ = optimizedPose;
        lastKeyframeMappedPose_ = optimizedPose;
        hasLastKeyframeMappedPose_ = true;

        PointType pose3D;
        pose3D.x = optimizedPose.translation.x();
        pose3D.y = optimizedPose.translation.y();
        pose3D.z = optimizedPose.translation.z();
        pose3D.intensity = static_cast<float>(key);
        cloudKeyPoses3D_->push_back(pose3D);

        PointTypePose pose6D;
        pose6D.x = pose3D.x;
        pose6D.y = pose3D.y;
        pose6D.z = pose3D.z;
        pose6D.intensity = pose3D.intensity;
        float roll, pitch, yaw;
        rpyFromRotation(optimizedPose.rotation, roll, pitch, yaw);
        pose6D.roll = roll;
        pose6D.pitch = pitch;
        pose6D.yaw = yaw;
        pose6D.time = timeLaserOdometry_;
        cloudKeyPoses6D_->push_back(pose6D);

        // Store keyframe data in the local body frame. It is transformed by
        // the optimized key pose whenever a local map is rebuilt.
        pcl::PointCloud<PointType>::Ptr corner(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr surf(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr outlier(new pcl::PointCloud<PointType>());
        *corner = *laserCloudCornerLastDS_;
        downSizeFilterSurf_.setInputCloud(laserCloudSurfLast_);
        downSizeFilterSurf_.filter(*surf);
        downSizeFilterOutlier_.setInputCloud(laserCloudOutlierLast_);
        downSizeFilterOutlier_.filter(*outlier);
        cornerCloudKeyFrames_.push_back(corner);
        surfCloudKeyFrames_.push_back(surf);
        outlierCloudKeyFrames_.push_back(outlier);
    }

    void publishOutputs()
    {
        const PoseState pose = currentMappedPose();
        const ros::Time stamp = ros::Time().fromSec(timeLaserOdometry_);

        nav_msgs::Odometry odometry;
        odometry.header.stamp = stamp;
        odometry.header.frame_id = "map";
        odometry.child_frame_id = "aft_mapped";
        odometry.pose.pose.position.x = pose.translation.x();
        odometry.pose.pose.position.y = pose.translation.y();
        odometry.pose.pose.position.z = pose.translation.z();
        const geometry_msgs::Quaternion q = tf::createQuaternionMsgFromRollPitchYaw(
            transformTobeMapped_[0], transformTobeMapped_[1], transformTobeMapped_[2]);
        odometry.pose.pose.orientation = q;
        pubOdomAftMapped_.publish(odometry);

        tf::Transform transform;
        transform.setOrigin(tf::Vector3(pose.translation.x(), pose.translation.y(), pose.translation.z()));
        transform.setRotation(tf::Quaternion(q.x, q.y, q.z, q.w));
        tfBroadcaster_.sendTransform(tf::StampedTransform(transform, stamp, "map", "aft_mapped"));

        if (pubKeyPoses_.getNumSubscribers() != 0) {
            sensor_msgs::PointCloud2 message;
            pcl::toROSMsg(*cloudKeyPoses3D_, message);
            message.header.stamp = stamp;
            message.header.frame_id = "map";
            pubKeyPoses_.publish(message);
        }

        if (pubLaserCloudSurround_.getNumSubscribers() != 0) {
            pcl::PointCloud<PointType>::Ptr localMap(new pcl::PointCloud<PointType>());
            *localMap += *laserCloudCornerFromMapDS_;
            *localMap += *laserCloudSurfFromMapDS_;
            sensor_msgs::PointCloud2 message;
            pcl::toROSMsg(*localMap, message);
            message.header.stamp = stamp;
            message.header.frame_id = "map";
            pubLaserCloudSurround_.publish(message);
        }

        if (pubRegisteredCloud_.getNumSubscribers() != 0) {
            pcl::PointCloud<PointType>::Ptr registered(new pcl::PointCloud<PointType>());
            *registered += *transformCloud(laserCloudCornerLastDS_, pose);
            *registered += *transformCloud(laserCloudSurfTotalLastDS_, pose);
            sensor_msgs::PointCloud2 message;
            pcl::toROSMsg(*registered, message);
            message.header.stamp = stamp;
            message.header.frame_id = "map";
            pubRegisteredCloud_.publish(message);
        }
    }

    void clearLocalMap()
    {
        laserCloudCornerFromMap_->clear();
        laserCloudSurfFromMap_->clear();
        laserCloudCornerFromMapDS_->clear();
        laserCloudSurfFromMapDS_->clear();
    }

    tf::TransformBroadcaster tfBroadcaster_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mapOptimization");
    ros::NodeHandle privateNode("~");
    lego_lio::configureLogger(privateNode);
    LOG_INFO << "Map Optimization Started (standard ROS FLU coordinates).";

    MapOptimization mapOptimization;
    ros::Rate rate(200);
    while (ros::ok()) {
        ros::spinOnce();
        mapOptimization.run();
        rate.sleep();
    }
    return 0;
}
