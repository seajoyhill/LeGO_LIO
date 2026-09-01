#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_

#define PCL_NO_PRECOMPILE

#include <algorithm>
#include <array>
#include <cmath>
#include <cfloat>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <mutex>
#include <queue>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <ros/ros.h>

#include "logger.hpp"

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include "lego_lio/cloud_info.h"

#include <opencv2/opencv.hpp>

// OpenCV defines this FLANN implementation switch globally. The FLANN build
// shipped by the current ROS/PCL environment has no unordered_map serializer,
// so select its supported std::map implementation before including PCL/FLANN.
#ifdef USE_UNORDERED_MAP
#undef USE_UNORDERED_MAP
#endif
#define USE_UNORDERED_MAP 0

#include <pcl/common/common.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#define PI 3.14159265

using namespace std;

typedef pcl::PointXYZI PointType;

static const string pointCloudTopic = "/velodyne_points";
static const string imuTopic = "/imu";
static const string fileDirectory = "/tmp/";
constexpr int N_SCAN = 32;
constexpr int Horizon_SCAN = 1800;
constexpr float ang_res_x = 0.2;
constexpr float ang_res_y = 1.0;
constexpr float ang_bottom = 16.0;
constexpr int groundScanInd = 17;

constexpr bool loopClosureEnableFlag = false;
constexpr double mappingProcessInterval = 0.3;

constexpr float scanPeriod = 0.1;
constexpr int systemDelay = 0;
constexpr int imuQueLength = 200;

constexpr float sensorMinimumRange = 1.0;
constexpr float sensorMountAngle = 0.0;
constexpr float segmentTheta = 60.0 / 180.0 * M_PI;
constexpr int segmentValidPointNum = 5;
constexpr int segmentValidLineNum = 3;
constexpr float segmentAlphaX = ang_res_x / 180.0 * M_PI;
constexpr float segmentAlphaY = ang_res_y / 180.0 * M_PI;

constexpr int edgeFeatureNum = 2;
constexpr int surfFeatureNum = 4;
constexpr int sectionsTotal = 6;
constexpr float edgeThreshold = 0.1;
constexpr float surfThreshold = 0.1;
constexpr float nearestFeatureSearchSqDist = 25;

constexpr float surroundingKeyframeSearchRadius = 50.0;
constexpr int surroundingKeyframeSearchNum = 50;
constexpr float historyKeyframeSearchRadius = 7.0;
constexpr int historyKeyframeSearchNum = 25;
constexpr float historyKeyframeFitnessScore = 0.3;
constexpr float globalMapVisualizationSearchRadius = 500.0;

struct smoothness_t
{
    float value;
    size_t ind;
};

struct by_value
{
    bool operator()(smoothness_t const& left, smoothness_t const& right)
    {
        return left.value < right.value;
    }
};

struct EIGEN_ALIGN16 PointXYZIR
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIR,
    (float, x, x) (float, y, y)
    (float, z, z) (float, intensity, intensity)
    (uint16_t, ring, ring)
)

struct EIGEN_ALIGN16 PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRPYT,
    (float, x, x) (float, y, y)
    (float, z, z) (float, intensity, intensity)
    (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
    (double, time, time)
)

typedef PointXYZIRPYT PointTypePose;

namespace lego_lio
{
inline std::string loggerNodeName()
{
    std::string name = ros::this_node::getName();
    while (!name.empty() && name.front() == '/')
        name.erase(name.begin());
    std::replace(name.begin(), name.end(), '/', '_');
    return name.empty() ? "node" : name;
}

inline std::string expandLoggerPath(std::string path)
{
    const std::string token = "{node}";
    const std::string nodeName = loggerNodeName();
    std::string::size_type position = 0;
    while ((position = path.find(token, position)) != std::string::npos) {
        path.replace(position, token.size(), nodeName);
        position += nodeName.size();
    }
    return path;
}

// Applies optional private ROS parameters under ~log. When a parameter is not
// present, the logger keeps its API/environment/default value.
inline bool configureLogger(const ros::NodeHandle& privateNode)
{
    log::Logger& logger = log::Logger::instance();
    bool valid = true;
    std::string value;
    bool flag = true;

    if (privateNode.getParam("log/level", value) && !logger.setLevel(value)) {
        ROS_WARN("Invalid ~log/level '%s'; keeping the previous level.", value.c_str());
        valid = false;
    }
    if (privateNode.getParam("log/console", flag))
        logger.setConsoleEnabled(flag);
    if (privateNode.getParam("log/flush_level", value) && !logger.setFlushLevel(value)) {
        ROS_WARN("Invalid ~log/flush_level '%s'; keeping the previous level.", value.c_str());
        valid = false;
    }

    bool append = true;
    privateNode.param("log/append", append, true);
    if (privateNode.getParam("log/file", value)) {
        value = expandLoggerPath(value);
        if (!logger.setFile(value, append)) {
            ROS_ERROR("Cannot open LeGO-LIO log file '%s'.", value.c_str());
            valid = false;
        }
    }
    return valid;
}
}  // namespace lego_lio

#endif
