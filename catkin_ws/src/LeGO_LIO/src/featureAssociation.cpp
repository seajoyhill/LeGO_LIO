// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is an implementation of the algorithm described in the following papers:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.
//   T. Shan and B. Englot. LeGO-LOAM: Lightweight and Ground-Optimized Lidar Odometry and Mapping on Variable Terrain
//      IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). October 2018.

#include "utility.h"

#include <Eigen/Geometry>

class FeatureAssociation{

private:

	ros::NodeHandle nh;

    ros::Subscriber subLaserCloud;
    ros::Subscriber subLaserCloudInfo;
    ros::Subscriber subOutlierCloud;
    ros::Subscriber subImu;

    ros::Publisher pubCornerPointsSharp;
    ros::Publisher pubCornerPointsLessSharp;
    ros::Publisher pubSurfPointsFlat;
    ros::Publisher pubSurfPointsLessFlat;

    pcl::PointCloud<PointType>::Ptr segmentedCloud;
    pcl::PointCloud<PointType>::Ptr outlierCloud;

    pcl::PointCloud<PointType>::Ptr cornerPointsSharp;
    pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp;
    pcl::PointCloud<PointType>::Ptr surfPointsFlat;
    pcl::PointCloud<PointType>::Ptr surfPointsLessFlat;

    pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan;
    pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScanDS;

    pcl::VoxelGrid<PointType> downSizeFilter;

    double timeScanCur;
    double timeNewSegmentedCloud;
    double timeNewSegmentedCloudInfo;
    double timeNewOutlierCloud;

    bool newSegmentedCloud;
    bool newSegmentedCloudInfo;
    bool newOutlierCloud;

    lego_lio::cloud_info segInfo;
    std_msgs::Header cloudHeader;

    int systemInitCount;
    bool systemInited;

    std::vector<smoothness_t> cloudSmoothness;
    float *cloudCurvature;
    int *cloudNeighborPicked;
    int *cloudLabel;

    int imuPointerFront;
    int imuPointerLast;
    int imuPointerLastIteration;

    float imuRollStart, imuPitchStart, imuYawStart;
    float cosImuRollStart, cosImuPitchStart, cosImuYawStart, sinImuRollStart, sinImuPitchStart, sinImuYawStart;
    float imuRollCur, imuPitchCur, imuYawCur;

    float imuVeloXStart, imuVeloYStart, imuVeloZStart;
    float imuShiftXStart, imuShiftYStart, imuShiftZStart;

    float imuVeloXCur, imuVeloYCur, imuVeloZCur;
    float imuShiftXCur, imuShiftYCur, imuShiftZCur;

    float imuShiftFromStartXCur, imuShiftFromStartYCur, imuShiftFromStartZCur;
    float imuVeloFromStartXCur, imuVeloFromStartYCur, imuVeloFromStartZCur;

    float imuAngularRotationXCur, imuAngularRotationYCur, imuAngularRotationZCur;
    float imuAngularRotationXLast, imuAngularRotationYLast, imuAngularRotationZLast;
    float imuAngularFromStartX, imuAngularFromStartY, imuAngularFromStartZ;

    double imuTime[imuQueLength];
    float imuRoll[imuQueLength];
    float imuPitch[imuQueLength];
    float imuYaw[imuQueLength];

    float imuAccX[imuQueLength];
    float imuAccY[imuQueLength];
    float imuAccZ[imuQueLength];

    float imuVeloX[imuQueLength];
    float imuVeloY[imuQueLength];
    float imuVeloZ[imuQueLength];

    float imuShiftX[imuQueLength];
    float imuShiftY[imuQueLength];
    float imuShiftZ[imuQueLength];

    float imuAngularVeloX[imuQueLength];
    float imuAngularVeloY[imuQueLength];
    float imuAngularVeloZ[imuQueLength];

    float imuAngularRotationX[imuQueLength];
    float imuAngularRotationY[imuQueLength];
    float imuAngularRotationZ[imuQueLength];



    ros::Publisher pubLaserCloudCornerLast;
    ros::Publisher pubLaserCloudSurfLast;
    ros::Publisher pubLaserOdometry;
    ros::Publisher pubOutlierCloudLast;

    int skipFrameNum;
    bool systemInitedLM;

    int laserCloudCornerLastNum;
    int laserCloudSurfLastNum;

    int *pointSelCornerInd;
    float *pointSearchCornerInd1;
    float *pointSearchCornerInd2;

    int *pointSelSurfInd;
    float *pointSearchSurfInd1;
    float *pointSearchSurfInd2;
    float *pointSearchSurfInd3;

    float transformCur[6];
    float transformSum[6];

    float imuRollLast, imuPitchLast, imuYawLast;
    float imuShiftFromStartX, imuShiftFromStartY, imuShiftFromStartZ;
    float imuVeloFromStartX, imuVeloFromStartY, imuVeloFromStartZ;

    pcl::PointCloud<PointType>::Ptr laserCloudCornerLast;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLast;
    pcl::PointCloud<PointType>::Ptr laserCloudOri;
    pcl::PointCloud<PointType>::Ptr coeffSel;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerLast;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfLast;

    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;

    PointType pointOri, pointSel, tripod1, tripod2, tripod3, pointProj, coeff;

    nav_msgs::Odometry laserOdometry;

    tf::TransformBroadcaster tfBroadcaster;
    tf::StampedTransform laserOdometryTrans;

    bool isDegenerate;
    cv::Mat matP;

    int frameCount;

public:

    FeatureAssociation():
        nh("~")
        {

        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/segmented_cloud", 1, &FeatureAssociation::laserCloudHandler, this);
        subLaserCloudInfo = nh.subscribe<lego_lio::cloud_info>("/segmented_cloud_info", 1, &FeatureAssociation::laserCloudInfoHandler, this);
        subOutlierCloud = nh.subscribe<sensor_msgs::PointCloud2>("/outlier_cloud", 1, &FeatureAssociation::outlierCloudHandler, this);
        subImu = nh.subscribe<sensor_msgs::Imu>(imuTopic, 50, &FeatureAssociation::imuHandler, this);

        pubCornerPointsSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 1);
        pubCornerPointsLessSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 1);
        pubSurfPointsFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_flat", 1);
        pubSurfPointsLessFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 1);

        pubLaserCloudCornerLast = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_corner_last", 2);
        pubLaserCloudSurfLast = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surf_last", 2);
        pubOutlierCloudLast = nh.advertise<sensor_msgs::PointCloud2>("/outlier_cloud_last", 2);
        pubLaserOdometry = nh.advertise<nav_msgs::Odometry> ("/laser_odom_to_init", 5);
        
        initializationValue();
    }

    void initializationValue()
    {
        cloudCurvature = new float[N_SCAN*Horizon_SCAN];
        cloudNeighborPicked = new int[N_SCAN*Horizon_SCAN];
        cloudLabel = new int[N_SCAN*Horizon_SCAN];

        pointSelCornerInd = new int[N_SCAN*Horizon_SCAN];
        pointSearchCornerInd1 = new float[N_SCAN*Horizon_SCAN];
        pointSearchCornerInd2 = new float[N_SCAN*Horizon_SCAN];

        pointSelSurfInd = new int[N_SCAN*Horizon_SCAN];
        pointSearchSurfInd1 = new float[N_SCAN*Horizon_SCAN];
        pointSearchSurfInd2 = new float[N_SCAN*Horizon_SCAN];
        pointSearchSurfInd3 = new float[N_SCAN*Horizon_SCAN];

        cloudSmoothness.resize(N_SCAN*Horizon_SCAN);

        downSizeFilter.setLeafSize(0.2, 0.2, 0.2);

        segmentedCloud.reset(new pcl::PointCloud<PointType>());
        outlierCloud.reset(new pcl::PointCloud<PointType>());

        cornerPointsSharp.reset(new pcl::PointCloud<PointType>());
        cornerPointsLessSharp.reset(new pcl::PointCloud<PointType>());
        surfPointsFlat.reset(new pcl::PointCloud<PointType>());
        surfPointsLessFlat.reset(new pcl::PointCloud<PointType>());

        surfPointsLessFlatScan.reset(new pcl::PointCloud<PointType>());
        surfPointsLessFlatScanDS.reset(new pcl::PointCloud<PointType>());

        timeScanCur = 0;
        timeNewSegmentedCloud = 0;
        timeNewSegmentedCloudInfo = 0;
        timeNewOutlierCloud = 0;

        newSegmentedCloud = false;
        newSegmentedCloudInfo = false;
        newOutlierCloud = false;

        systemInitCount = 0;
        systemInited = false;

        imuPointerFront = 0;
        imuPointerLast = -1;
        imuPointerLastIteration = 0;

        imuRollStart = 0; imuPitchStart = 0; imuYawStart = 0;
        cosImuRollStart = 0; cosImuPitchStart = 0; cosImuYawStart = 0;
        sinImuRollStart = 0; sinImuPitchStart = 0; sinImuYawStart = 0;
        imuRollCur = 0; imuPitchCur = 0; imuYawCur = 0;

        imuVeloXStart = 0; imuVeloYStart = 0; imuVeloZStart = 0;
        imuShiftXStart = 0; imuShiftYStart = 0; imuShiftZStart = 0;

        imuVeloXCur = 0; imuVeloYCur = 0; imuVeloZCur = 0;
        imuShiftXCur = 0; imuShiftYCur = 0; imuShiftZCur = 0;

        imuShiftFromStartXCur = 0; imuShiftFromStartYCur = 0; imuShiftFromStartZCur = 0;
        imuVeloFromStartXCur = 0; imuVeloFromStartYCur = 0; imuVeloFromStartZCur = 0;

        imuAngularRotationXCur = 0; imuAngularRotationYCur = 0; imuAngularRotationZCur = 0;
        imuAngularRotationXLast = 0; imuAngularRotationYLast = 0; imuAngularRotationZLast = 0;
        imuAngularFromStartX = 0; imuAngularFromStartY = 0; imuAngularFromStartZ = 0;

        for (int i = 0; i < imuQueLength; ++i)
        {
            imuTime[i] = 0;
            imuRoll[i] = 0; imuPitch[i] = 0; imuYaw[i] = 0;
            imuAccX[i] = 0; imuAccY[i] = 0; imuAccZ[i] = 0;
            imuVeloX[i] = 0; imuVeloY[i] = 0; imuVeloZ[i] = 0;
            imuShiftX[i] = 0; imuShiftY[i] = 0; imuShiftZ[i] = 0;
            imuAngularVeloX[i] = 0; imuAngularVeloY[i] = 0; imuAngularVeloZ[i] = 0;
            imuAngularRotationX[i] = 0; imuAngularRotationY[i] = 0; imuAngularRotationZ[i] = 0;
        }


        skipFrameNum = 1;

        for (int i = 0; i < 6; ++i){
            transformCur[i] = 0;
            transformSum[i] = 0;
        }

        systemInitedLM = false;

        imuRollLast = 0; imuPitchLast = 0; imuYawLast = 0;
        imuShiftFromStartX = 0; imuShiftFromStartY = 0; imuShiftFromStartZ = 0;
        imuVeloFromStartX = 0; imuVeloFromStartY = 0; imuVeloFromStartZ = 0;

        laserCloudCornerLast.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfLast.reset(new pcl::PointCloud<PointType>());
        laserCloudOri.reset(new pcl::PointCloud<PointType>());
        coeffSel.reset(new pcl::PointCloud<PointType>());

        kdtreeCornerLast.reset(new pcl::KdTreeFLANN<PointType>());
        kdtreeSurfLast.reset(new pcl::KdTreeFLANN<PointType>());

        laserOdometry.header.frame_id = "odom";
        laserOdometry.child_frame_id = "base_link";

        laserOdometryTrans.frame_id_ = "odom";
        laserOdometryTrans.child_frame_id_ = "base_link";
        
        isDegenerate = false;
        matP = cv::Mat(6, 6, CV_32F, cv::Scalar::all(0));

        frameCount = skipFrameNum;
    }

    Eigen::Matrix3f rotationMatrix(float roll, float pitch, float yaw) const
    {
        // Standard ROS fixed-axis RPY: R = Rz(yaw) * Ry(pitch) * Rx(roll).
        return (Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()) *
                Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
                Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX())).toRotationMatrix();
    }

    void matrixToRPY(const Eigen::Matrix3f& rotation,
                     float& roll, float& pitch, float& yaw) const
    {
        const float sinPitch = std::max(-1.0f, std::min(1.0f, -rotation(2, 0)));
        pitch = std::asin(sinPitch);

        if (std::abs(std::cos(pitch)) > 1e-6f) {
            roll = std::atan2(rotation(2, 1), rotation(2, 2));
            yaw = std::atan2(rotation(1, 0), rotation(0, 0));
        } else {
            // Gimbal-lock fallback: keep a valid equivalent RPY representation.
            roll = 0.0f;
            yaw = std::atan2(-rotation(0, 1), rotation(1, 1));
        }
    }

    Eigen::Vector3f transformPointToStart(const PointType& point,
                                           const float transform[6]) const
    {
        const float s = scanPeriod > 0.0f
                      ? (point.intensity - std::floor(point.intensity)) / scanPeriod
                      : 0.0f;
        const Eigen::Matrix3f rotation = rotationMatrix(
            s * transform[0], s * transform[1], s * transform[2]);
        const Eigen::Vector3f translation(
            s * transform[3], s * transform[4], s * transform[5]);
        const Eigen::Vector3f p(point.x, point.y, point.z);
        return rotation.transpose() * (p - translation);
    }

    void updateImuRollPitchYawStartSinCos()
    {
        // Kept for source compatibility with the original call sites. All new
        // coordinate-correct IMU math uses rotation matrices directly.
        cosImuRollStart = cos(imuRollStart);
        cosImuPitchStart = cos(imuPitchStart);
        cosImuYawStart = cos(imuYawStart);
        sinImuRollStart = sin(imuRollStart);
        sinImuPitchStart = sin(imuPitchStart);
        sinImuYawStart = sin(imuYawStart);
    }

    void ShiftToStartIMU(float pointTime)
    {
        const Eigen::Vector3f deltaWorld(
            imuShiftXCur - imuShiftXStart - imuVeloXStart * pointTime,
            imuShiftYCur - imuShiftYStart - imuVeloYStart * pointTime,
            imuShiftZCur - imuShiftZStart - imuVeloZStart * pointTime);

        const Eigen::Vector3f deltaStart =
            rotationMatrix(imuRollStart, imuPitchStart, imuYawStart).transpose() * deltaWorld;
        imuShiftFromStartXCur = deltaStart.x();
        imuShiftFromStartYCur = deltaStart.y();
        imuShiftFromStartZCur = deltaStart.z();
    }

    void VeloToStartIMU()
    {
        const Eigen::Vector3f deltaWorld(
            imuVeloXCur - imuVeloXStart,
            imuVeloYCur - imuVeloYStart,
            imuVeloZCur - imuVeloZStart);

        const Eigen::Vector3f deltaStart =
            rotationMatrix(imuRollStart, imuPitchStart, imuYawStart).transpose() * deltaWorld;
        imuVeloFromStartXCur = deltaStart.x();
        imuVeloFromStartYCur = deltaStart.y();
        imuVeloFromStartZCur = deltaStart.z();
    }

    void TransformToStartIMU(PointType *point)
    {
        const Eigen::Matrix3f rotationStart =
            rotationMatrix(imuRollStart, imuPitchStart, imuYawStart);
        const Eigen::Matrix3f rotationCur =
            rotationMatrix(imuRollCur, imuPitchCur, imuYawCur);
        const Eigen::Vector3f shiftStart(
            imuShiftFromStartXCur, imuShiftFromStartYCur, imuShiftFromStartZCur);
        const Eigen::Vector3f p(point->x, point->y, point->z);
        const Eigen::Vector3f pStart = rotationStart.transpose() * rotationCur * p + shiftStart;

        point->x = pStart.x();
        point->y = pStart.y();
        point->z = pStart.z();
    }

    void AccumulateIMUShiftAndRotation()
    {
        const Eigen::Vector3f accelerationBody(
            imuAccX[imuPointerLast], imuAccY[imuPointerLast], imuAccZ[imuPointerLast]);
        const Eigen::Vector3f accelerationWorld =
            rotationMatrix(imuRoll[imuPointerLast],
                           imuPitch[imuPointerLast],
                           imuYaw[imuPointerLast]) * accelerationBody;

        const int imuPointerBack = (imuPointerLast + imuQueLength - 1) % imuQueLength;
        const double timeDiff = imuTime[imuPointerLast] - imuTime[imuPointerBack];
        if (timeDiff > 0.0 && timeDiff < scanPeriod) {
            imuShiftX[imuPointerLast] = imuShiftX[imuPointerBack] +
                imuVeloX[imuPointerBack] * timeDiff + accelerationWorld.x() * timeDiff * timeDiff / 2.0;
            imuShiftY[imuPointerLast] = imuShiftY[imuPointerBack] +
                imuVeloY[imuPointerBack] * timeDiff + accelerationWorld.y() * timeDiff * timeDiff / 2.0;
            imuShiftZ[imuPointerLast] = imuShiftZ[imuPointerBack] +
                imuVeloZ[imuPointerBack] * timeDiff + accelerationWorld.z() * timeDiff * timeDiff / 2.0;

            imuVeloX[imuPointerLast] = imuVeloX[imuPointerBack] + accelerationWorld.x() * timeDiff;
            imuVeloY[imuPointerLast] = imuVeloY[imuPointerBack] + accelerationWorld.y() * timeDiff;
            imuVeloZ[imuPointerLast] = imuVeloZ[imuPointerBack] + accelerationWorld.z() * timeDiff;

            imuAngularRotationX[imuPointerLast] = imuAngularRotationX[imuPointerBack] +
                imuAngularVeloX[imuPointerBack] * timeDiff;
            imuAngularRotationY[imuPointerLast] = imuAngularRotationY[imuPointerBack] +
                imuAngularVeloY[imuPointerBack] * timeDiff;
            imuAngularRotationZ[imuPointerLast] = imuAngularRotationZ[imuPointerBack] +
                imuAngularVeloZ[imuPointerBack] * timeDiff;
        }
    }

    void imuHandler(const sensor_msgs::Imu::ConstPtr& imuIn)
    {
        double roll, pitch, yaw;
        tf::Quaternion orientation;
        tf::quaternionMsgToTF(imuIn->orientation, orientation);
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

        // 重力补偿：令 roll=\phi、pitch=\theta，g=9.81 m/s^2。
        // rotationMatrix() 采用 R=R_z(yaw)R_y(pitch)R_x(roll)，将机体系向量映射到世界系。
        // 世界系重力为 g_w=[0, 0, -g]^T，因此机体系中的重力为：
        //   g_b = R^T g_w
        //       = [-g sin(theta), g sin(phi) cos(theta), -g cos(phi) cos(theta)]^T
        // sensor_msgs/Imu::linear_acceleration 在此按比力 f_b=a_b-g_b 解释，
        // 非重力线加速度由 a_b=f_b+g_b 得到。由于 g_w 沿世界系 z 轴，yaw 不出现在该式中。
        const float accX = imuIn->linear_acceleration.x + std::sin(pitch) * 9.81f;
        const float accY = imuIn->linear_acceleration.y -
                           std::sin(roll) * std::cos(pitch) * 9.81f;
        const float accZ = imuIn->linear_acceleration.z -
                           std::cos(roll) * std::cos(pitch) * 9.81f;

        imuPointerLast = (imuPointerLast + 1) % imuQueLength;
        imuTime[imuPointerLast] = imuIn->header.stamp.toSec();
        imuRoll[imuPointerLast] = roll;
        imuPitch[imuPointerLast] = pitch;
        imuYaw[imuPointerLast] = yaw;
        imuAccX[imuPointerLast] = accX;
        imuAccY[imuPointerLast] = accY;
        imuAccZ[imuPointerLast] = accZ;
        imuAngularVeloX[imuPointerLast] = imuIn->angular_velocity.x;
        imuAngularVeloY[imuPointerLast] = imuIn->angular_velocity.y;
        imuAngularVeloZ[imuPointerLast] = imuIn->angular_velocity.z;

        AccumulateIMUShiftAndRotation();
    }

    void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){

        cloudHeader = laserCloudMsg->header;

        timeScanCur = cloudHeader.stamp.toSec();
        timeNewSegmentedCloud = timeScanCur;

        segmentedCloud->clear();
        pcl::fromROSMsg(*laserCloudMsg, *segmentedCloud);

        newSegmentedCloud = true;
    }

    void outlierCloudHandler(const sensor_msgs::PointCloud2ConstPtr& msgIn){

        timeNewOutlierCloud = msgIn->header.stamp.toSec();

        outlierCloud->clear();
        pcl::fromROSMsg(*msgIn, *outlierCloud);

        newOutlierCloud = true;
    }

    void laserCloudInfoHandler(const lego_lio::cloud_infoConstPtr& msgIn)
    {
        timeNewSegmentedCloudInfo = msgIn->header.stamp.toSec();
        segInfo = *msgIn;
        newSegmentedCloudInfo = true;
    }

    void adjustDistortion()
    {
        bool halfPassed = false;
        int cloudSize = segmentedCloud->points.size();

        PointType point;

        for (int i = 0; i < cloudSize; i++) {

            // Keep the native ROS/REP-103 FLU coordinates:
            // x forward, y left, z up.
            point = segmentedCloud->points[i];

            float ori = -atan2(point.y, point.x);
            if (!halfPassed) {
                if (ori < segInfo.startOrientation - M_PI / 2)
                    ori += 2 * M_PI;
                else if (ori > segInfo.startOrientation + M_PI * 3 / 2)
                    ori -= 2 * M_PI;

                if (ori - segInfo.startOrientation > M_PI)
                    halfPassed = true;
            } else {
                ori += 2 * M_PI;

                if (ori < segInfo.endOrientation - M_PI * 3 / 2)
                    ori += 2 * M_PI;
                else if (ori > segInfo.endOrientation + M_PI / 2)
                    ori -= 2 * M_PI;
            }

            float relTime = (ori - segInfo.startOrientation) / segInfo.orientationDiff;
            point.intensity = int(segmentedCloud->points[i].intensity) + scanPeriod * relTime;

            if (imuPointerLast >= 0) {
                float pointTime = relTime * scanPeriod;
                imuPointerFront = imuPointerLastIteration;
                while (imuPointerFront != imuPointerLast) {
                    if (timeScanCur + pointTime < imuTime[imuPointerFront]) {
                        break;
                    }
                    imuPointerFront = (imuPointerFront + 1) % imuQueLength;
                }

                if (timeScanCur + pointTime > imuTime[imuPointerFront]) {
                    imuRollCur = imuRoll[imuPointerFront];
                    imuPitchCur = imuPitch[imuPointerFront];
                    imuYawCur = imuYaw[imuPointerFront];

                    imuVeloXCur = imuVeloX[imuPointerFront];
                    imuVeloYCur = imuVeloY[imuPointerFront];
                    imuVeloZCur = imuVeloZ[imuPointerFront];

                    imuShiftXCur = imuShiftX[imuPointerFront];
                    imuShiftYCur = imuShiftY[imuPointerFront];
                    imuShiftZCur = imuShiftZ[imuPointerFront];   
                } else {
                    int imuPointerBack = (imuPointerFront + imuQueLength - 1) % imuQueLength;
                    float ratioFront = (timeScanCur + pointTime - imuTime[imuPointerBack]) 
                                                     / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
                    float ratioBack = (imuTime[imuPointerFront] - timeScanCur - pointTime) 
                                                    / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);

                    imuRollCur = imuRoll[imuPointerFront] * ratioFront + imuRoll[imuPointerBack] * ratioBack;
                    imuPitchCur = imuPitch[imuPointerFront] * ratioFront + imuPitch[imuPointerBack] * ratioBack;
                    if (imuYaw[imuPointerFront] - imuYaw[imuPointerBack] > M_PI) {
                        imuYawCur = imuYaw[imuPointerFront] * ratioFront + (imuYaw[imuPointerBack] + 2 * M_PI) * ratioBack;
                    } else if (imuYaw[imuPointerFront] - imuYaw[imuPointerBack] < -M_PI) {
                        imuYawCur = imuYaw[imuPointerFront] * ratioFront + (imuYaw[imuPointerBack] - 2 * M_PI) * ratioBack;
                    } else {
                        imuYawCur = imuYaw[imuPointerFront] * ratioFront + imuYaw[imuPointerBack] * ratioBack;
                    }

                    imuVeloXCur = imuVeloX[imuPointerFront] * ratioFront + imuVeloX[imuPointerBack] * ratioBack;
                    imuVeloYCur = imuVeloY[imuPointerFront] * ratioFront + imuVeloY[imuPointerBack] * ratioBack;
                    imuVeloZCur = imuVeloZ[imuPointerFront] * ratioFront + imuVeloZ[imuPointerBack] * ratioBack;

                    imuShiftXCur = imuShiftX[imuPointerFront] * ratioFront + imuShiftX[imuPointerBack] * ratioBack;
                    imuShiftYCur = imuShiftY[imuPointerFront] * ratioFront + imuShiftY[imuPointerBack] * ratioBack;
                    imuShiftZCur = imuShiftZ[imuPointerFront] * ratioFront + imuShiftZ[imuPointerBack] * ratioBack;
                }

                if (i == 0) {
                    imuRollStart = imuRollCur;
                    imuPitchStart = imuPitchCur;
                    imuYawStart = imuYawCur;

                    imuVeloXStart = imuVeloXCur;
                    imuVeloYStart = imuVeloYCur;
                    imuVeloZStart = imuVeloZCur;

                    imuShiftXStart = imuShiftXCur;
                    imuShiftYStart = imuShiftYCur;
                    imuShiftZStart = imuShiftZCur;

                    if (timeScanCur + pointTime > imuTime[imuPointerFront]) {
                        imuAngularRotationXCur = imuAngularRotationX[imuPointerFront];
                        imuAngularRotationYCur = imuAngularRotationY[imuPointerFront];
                        imuAngularRotationZCur = imuAngularRotationZ[imuPointerFront];
                    }else{
                        int imuPointerBack = (imuPointerFront + imuQueLength - 1) % imuQueLength;
                        float ratioFront = (timeScanCur + pointTime - imuTime[imuPointerBack]) 
                                                         / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
                        float ratioBack = (imuTime[imuPointerFront] - timeScanCur - pointTime) 
                                                        / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
                        imuAngularRotationXCur = imuAngularRotationX[imuPointerFront] * ratioFront + imuAngularRotationX[imuPointerBack] * ratioBack;
                        imuAngularRotationYCur = imuAngularRotationY[imuPointerFront] * ratioFront + imuAngularRotationY[imuPointerBack] * ratioBack;
                        imuAngularRotationZCur = imuAngularRotationZ[imuPointerFront] * ratioFront + imuAngularRotationZ[imuPointerBack] * ratioBack;
                    }

                    imuAngularFromStartX = imuAngularRotationXCur - imuAngularRotationXLast;
                    imuAngularFromStartY = imuAngularRotationYCur - imuAngularRotationYLast;
                    imuAngularFromStartZ = imuAngularRotationZCur - imuAngularRotationZLast;

                    imuAngularRotationXLast = imuAngularRotationXCur;
                    imuAngularRotationYLast = imuAngularRotationYCur;
                    imuAngularRotationZLast = imuAngularRotationZCur;

                    updateImuRollPitchYawStartSinCos();
                } else {
                    VeloToStartIMU();
                    TransformToStartIMU(&point);
                }
            }
            segmentedCloud->points[i] = point;
        }

        imuPointerLastIteration = imuPointerLast;
    }

    void calculateSmoothness()
    {
        int cloudSize = segmentedCloud->points.size();
        for (int i = 5; i < cloudSize - 5; i++) {

            float diffRange = segInfo.segmentedCloudRange[i-5] + segInfo.segmentedCloudRange[i-4]
                            + segInfo.segmentedCloudRange[i-3] + segInfo.segmentedCloudRange[i-2]
                            + segInfo.segmentedCloudRange[i-1] - segInfo.segmentedCloudRange[i] * 10
                            + segInfo.segmentedCloudRange[i+1] + segInfo.segmentedCloudRange[i+2]
                            + segInfo.segmentedCloudRange[i+3] + segInfo.segmentedCloudRange[i+4]
                            + segInfo.segmentedCloudRange[i+5];            

            cloudCurvature[i] = diffRange*diffRange;

            cloudNeighborPicked[i] = 0;
            cloudLabel[i] = 0;

            cloudSmoothness[i].value = cloudCurvature[i];
            cloudSmoothness[i].ind = i;
        }
    }

    // 标记两类不可靠的点，cloudNeighborPicked = 1 的点在 extractFeatures() 中会被跳过：
    // 1) 遮挡点：深度突变的远侧边缘，激光束掠射测量，坐标不可靠（假边缘点）
    // 2) 平行光束点：激光束几乎与表面平行，该点横向位置飘移大
    // 标记 ±5 邻域是因为 calculateSmoothness() 的曲率窗口为前后各 5 个点，
    // 避免坏点污染邻点的曲率计算。
    void markOccludedPoints()
    {
        int cloudSize = segmentedCloud->points.size();

        for (int i = 5; i < cloudSize - 6; ++i){

            float depth1 = segInfo.segmentedCloudRange[i];
            float depth2 = segInfo.segmentedCloudRange[i+1];
            // 点云按列（水平角）排序，相邻索引通常对应相邻列；
            // 列差 < 10 保证两者方位角接近，深度突变不是由列跳跃造成的
            int columnDiff = std::abs(int(segInfo.segmentedCloudColInd[i+1] - segInfo.segmentedCloudColInd[i]));

            if (columnDiff < 10){

                // 遮挡边界检测：相邻两点深度差超过 0.3m 时，
                // 远侧的点是激光擦过近处物体边缘斜着测到的，测距误差大，
                // 且下一帧可能扫不到，无法稳定匹配。近侧点保留，远侧点标记。
                if (depth1 - depth2 > 0.3){
                    cloudNeighborPicked[i - 5] = 1;
                    cloudNeighborPicked[i - 4] = 1;
                    cloudNeighborPicked[i - 3] = 1;
                    cloudNeighborPicked[i - 2] = 1;
                    cloudNeighborPicked[i - 1] = 1;
                    cloudNeighborPicked[i] = 1;
                }else if (depth2 - depth1 > 0.3){
                    cloudNeighborPicked[i + 1] = 1;
                    cloudNeighborPicked[i + 2] = 1;
                    cloudNeighborPicked[i + 3] = 1;
                    cloudNeighborPicked[i + 4] = 1;
                    cloudNeighborPicked[i + 5] = 1;
                    cloudNeighborPicked[i + 6] = 1;
                }
            }

            float diff1 = std::abs(float(segInfo.segmentedCloudRange[i-1] - segInfo.segmentedCloudRange[i]));
            float diff2 = std::abs(float(segInfo.segmentedCloudRange[i+1] - segInfo.segmentedCloudRange[i]));

            // 相对前后邻点的深度差都超过自身距离的 2% → 光束与表面近似平行
            if (diff1 > 0.02 * segInfo.segmentedCloudRange[i] && diff2 > 0.02 * segInfo.segmentedCloudRange[i])
                cloudNeighborPicked[i] = 1;
        }
    }

    // 曲率特征提取。按「扫描线(环) × 6 个子区间」划分, 每个子区间内
    // 按曲率排序后分类挑选, 向后续链路输出四类特征点:
    //   - cornerPointsSharp / cornerPointsLessSharp: 非地面点中的尖锐角点(点-线残差约束)
    //   - surfPointsFlat: 地面点中最平坦的点(点-面残差约束)
    //   - surfPointsLessFlat: 其余剩余点的降采样集合(宽松面特征池, 供点-面匹配兜底)
    // cloudLabel 约定: 2 = 最尖锐角点, 1 = 次角点, -1 = 平坦地面点, 0 = 未标记/普通点
    void extractFeatures()
    {
        cornerPointsSharp->clear();
        cornerPointsLessSharp->clear();
        surfPointsFlat->clear();
        surfPointsLessFlat->clear();

        // 外层: 每条扫描线(环)独立处理, 保证各环特征配额互不干扰
        for (int i = 0; i < N_SCAN; i++) {

            // 每环的剩余面点先清空, 6 个子区间累计完后再统一降采样
            surfPointsLessFlatScan->clear();

            // 中层: 把本环索引区间等分成 6 个子区间
            for (int j = 0; j < 6; j++) {

                // 插值的是「段边界索引」, sp/ep 是第 j 个子区间的起止下标:
                //   sp = 加权平均: S 权重 (6-j)/6, E 权重 j/6
                //      = 线性插值 S + (E-S)*j/6, 即从 S(startRingIndex) 向
                //        E(endRingIndex) 走 j/6 处;
                //   ep = 把 j 换成 j+1 代入 sp 公式再减 1, 恒有 ep = sp_{j+1} - 1,
                //       因此 6 段首尾相接、不重不漏地覆盖 [S, E-1]。
                // 意义: 各子区间独立挑特征并配给固定名额, 使角点/面点沿
                //       环的 360° 均匀分布, 而非全部挤在曲率最大的局部。
                int sp = (segInfo.startRingIndex[i] * (6 - j)    + segInfo.endRingIndex[i] * j) / 6;
                int ep = (segInfo.startRingIndex[i] * (5 - j)    + segInfo.endRingIndex[i] * (j + 1)) / 6 - 1;

                // 该环有效点过少时子区间长度不足(或只剩一个点), 直接跳过
                if (sp >= ep)
                    continue;

                // by_value 按曲率值升序: 最小值在左(sp 端), 最大值在右(ep 端)
                std::sort(cloudSmoothness.begin()+sp, cloudSmoothness.begin()+ep, by_value());

                int largestPickedNum = 0;
                // 角点候选: 未被遮挡标记、曲率足够尖锐(> edgeThreshold)且「非地面」的点。
                // 从曲率最高端(ep)向低端扫描, 段内配额: 前 2 个标为 sharp
                // (cloudLabel=2), 其后至多 18 个标为 less sharp(cloudLabel=1), 满 20 即止。
                for (int k = ep; k >= sp; k--) {
                    int ind = cloudSmoothness[k].ind;
                    if (cloudNeighborPicked[ind] == 0 &&
                        cloudCurvature[ind] > edgeThreshold &&
                        segInfo.segmentedCloudGroundFlag[ind] == false) {
                    
                        largestPickedNum++;
                        if (largestPickedNum <= 2) {
                            cloudLabel[ind] = 2;
                            cornerPointsSharp->push_back(segmentedCloud->points[ind]);
                            cornerPointsLessSharp->push_back(segmentedCloud->points[ind]);
                        } else if (largestPickedNum <= 20) {
                            cloudLabel[ind] = 1;
                            cornerPointsLessSharp->push_back(segmentedCloud->points[ind]);
                        } else {
                            break;
                        }

                        // 选中后把同列区(列差 ≤ 10)内的 ±5 个邻点一并标记,
                        // 防止一条竖向边缘上的相邻特征点扎堆, 造成特征冗余
                        cloudNeighborPicked[ind] = 1;
                        for (int l = 1; l <= 5; l++) {
                            int columnDiff = std::abs(int(segInfo.segmentedCloudColInd[ind + l] - segInfo.segmentedCloudColInd[ind + l - 1]));
                            if (columnDiff > 10)
                                break;
                            cloudNeighborPicked[ind + l] = 1;
                        }
                        for (int l = -1; l >= -5; l--) {
                            int columnDiff = std::abs(int(segInfo.segmentedCloudColInd[ind + l] - segInfo.segmentedCloudColInd[ind + l + 1]));
                            if (columnDiff > 10)
                                break;
                            cloudNeighborPicked[ind + l] = 1;
                        }
                    }
                }

                int smallestPickedNum = 0;
                // 面点候选: 未标记、曲率足够低(< surfThreshold)且「是地面」的点。
                // 与角点相反, 从曲率最低端(sp)向上扫描, 每段最多取 4 个
                // 最平坦的地面点标为 flat(cloudLabel=-1)。
                for (int k = sp; k <= ep; k++) {
                    int ind = cloudSmoothness[k].ind;
                    if (cloudNeighborPicked[ind] == 0 &&
                        cloudCurvature[ind] < surfThreshold &&
                        segInfo.segmentedCloudGroundFlag[ind] == true) {

                        cloudLabel[ind] = -1;
                        surfPointsFlat->push_back(segmentedCloud->points[ind]);

                        smallestPickedNum++;
                        if (smallestPickedNum >= 4) {
                            break;
                        }

                        // 同角点: 标记选中面点同列区 ±5 邻域, 避免平坦特征聚堆
                        cloudNeighborPicked[ind] = 1;
                        for (int l = 1; l <= 5; l++) {

                            int columnDiff = std::abs(int(segInfo.segmentedCloudColInd[ind + l] - segInfo.segmentedCloudColInd[ind + l - 1]));
                            if (columnDiff > 10)
                                break;

                            cloudNeighborPicked[ind + l] = 1;
                        }
                        for (int l = -1; l >= -5; l--) {

                            int columnDiff = std::abs(int(segInfo.segmentedCloudColInd[ind + l] - segInfo.segmentedCloudColInd[ind + l + 1]));
                            if (columnDiff > 10)
                                break;

                            cloudNeighborPicked[ind + l] = 1;
                        }
                    }
                }

                // 剩余点: cloudLabel <= 0 的全部流入该环的 lessFlat 集合——
                // 含未选角点/面点的普通点, 也包括 cloudLabel == -1 的地面点
                // (它们已进 surfPointsFlat, 又因 -1 <= 0 被重复计入; 原版即如此,
                //  lessFlat 是后续点-面匹配的宽松面特征池, 重复冗余无害)。
                for (int k = sp; k <= ep; k++) {
                    if (cloudLabel[k] <= 0) {
                        surfPointsLessFlatScan->push_back(segmentedCloud->points[k]);
                    }
                }
            }

            // 剩余点数量大, 体素降采样控制特征集尺寸后, 再并入总的 lessFlat
            surfPointsLessFlatScanDS->clear();
            downSizeFilter.setInputCloud(surfPointsLessFlatScan);
            downSizeFilter.filter(*surfPointsLessFlatScanDS);

            *surfPointsLessFlat += *surfPointsLessFlatScanDS;
        }
    }

    void publishCloud()
    {
        sensor_msgs::PointCloud2 laserCloudOutMsg;

	    if (pubCornerPointsSharp.getNumSubscribers() != 0){
	        pcl::toROSMsg(*cornerPointsSharp, laserCloudOutMsg);
	        laserCloudOutMsg.header.stamp = cloudHeader.stamp;
	        laserCloudOutMsg.header.frame_id = "base_link";
	        pubCornerPointsSharp.publish(laserCloudOutMsg);
	    }

	    if (pubCornerPointsLessSharp.getNumSubscribers() != 0){
	        pcl::toROSMsg(*cornerPointsLessSharp, laserCloudOutMsg);
	        laserCloudOutMsg.header.stamp = cloudHeader.stamp;
	        laserCloudOutMsg.header.frame_id = "base_link";
	        pubCornerPointsLessSharp.publish(laserCloudOutMsg);
	    }

	    if (pubSurfPointsFlat.getNumSubscribers() != 0){
	        pcl::toROSMsg(*surfPointsFlat, laserCloudOutMsg);
	        laserCloudOutMsg.header.stamp = cloudHeader.stamp;
	        laserCloudOutMsg.header.frame_id = "base_link";
	        pubSurfPointsFlat.publish(laserCloudOutMsg);
	    }

	    if (pubSurfPointsLessFlat.getNumSubscribers() != 0){
	        pcl::toROSMsg(*surfPointsLessFlat, laserCloudOutMsg);
	        laserCloudOutMsg.header.stamp = cloudHeader.stamp;
	        laserCloudOutMsg.header.frame_id = "base_link";
	        pubSurfPointsLessFlat.publish(laserCloudOutMsg);
	    }
    }










































    void TransformToStart(PointType const * const pi, PointType * const po)
    {
        const Eigen::Vector3f pStart = transformPointToStart(*pi, transformCur);
        po->x = pStart.x();
        po->y = pStart.y();
        po->z = pStart.z();
        po->intensity = pi->intensity;
    }

    void TransformToEnd(PointType const * const pi, PointType * const po)
    {
        // First deskew the point to scan start with the lidar-estimated motion.
        const Eigen::Vector3f pStart = transformPointToStart(*pi, transformCur);
        const Eigen::Matrix3f rotationEnd = rotationMatrix(
            transformCur[0], transformCur[1], transformCur[2]);
        const Eigen::Vector3f translationEnd(
            transformCur[3], transformCur[4], transformCur[5]);
        Eigen::Vector3f pEnd = rotationEnd * pStart + translationEnd;

        // Apply the IMU start-to-end correction in the same FLU frame.
        const Eigen::Matrix3f rotationImuStart =
            rotationMatrix(imuRollStart, imuPitchStart, imuYawStart);
        const Eigen::Matrix3f rotationImuEnd =
            rotationMatrix(imuRollLast, imuPitchLast, imuYawLast);
        const Eigen::Vector3f imuShift(
            imuShiftFromStartX, imuShiftFromStartY, imuShiftFromStartZ);
        pEnd = rotationImuEnd.transpose() * rotationImuStart * (pEnd - imuShift);

        po->x = pEnd.x();
        po->y = pEnd.y();
        po->z = pEnd.z();
        po->intensity = static_cast<int>(pi->intensity);
    }

    void PluginIMURotation(float baseRoll, float basePitch, float baseYaw,
                           float imuStartRoll, float imuStartPitch, float imuStartYaw,
                           float imuEndRoll, float imuEndPitch, float imuEndYaw,
                           float &outRoll, float &outPitch, float &outYaw)
    {
        const Eigen::Matrix3f rotationBase =
            rotationMatrix(baseRoll, basePitch, baseYaw);
        const Eigen::Matrix3f rotationImuStart =
            rotationMatrix(imuStartRoll, imuStartPitch, imuStartYaw);
        const Eigen::Matrix3f rotationImuEnd =
            rotationMatrix(imuEndRoll, imuEndPitch, imuEndYaw);
        matrixToRPY(rotationBase * rotationImuStart.transpose() * rotationImuEnd,
                    outRoll, outPitch, outYaw);
    }

    void AccumulateRotation(float currentRoll, float currentPitch, float currentYaw,
                            float incrementRoll, float incrementPitch, float incrementYaw,
                            float &outRoll, float &outPitch, float &outYaw)
    {
        matrixToRPY(rotationMatrix(currentRoll, currentPitch, currentYaw) *
                    rotationMatrix(incrementRoll, incrementPitch, incrementYaw),
                    outRoll, outPitch, outYaw);
    }

    double rad2deg(double radians)
    {
        return radians * 180.0 / M_PI;
    }

    double deg2rad(double degrees)
    {
        return degrees * M_PI / 180.0;
    }

    void findCorrespondingCornerFeatures(int iterCount){

        int cornerPointsSharpNum = cornerPointsSharp->points.size();

        for (int i = 0; i < cornerPointsSharpNum; i++) {

            TransformToStart(&cornerPointsSharp->points[i], &pointSel);

            if (iterCount % 5 == 0) {

                kdtreeCornerLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);
                int closestPointInd = -1, minPointInd2 = -1;
                
                if (pointSearchSqDis[0] < nearestFeatureSearchSqDist) {
                    closestPointInd = pointSearchInd[0];
                    int closestPointScan = int(laserCloudCornerLast->points[closestPointInd].intensity);

                    float pointSqDis, minPointSqDis2 = nearestFeatureSearchSqDist;
                    for (int j = closestPointInd + 1; j < cornerPointsSharpNum; j++) {
                        if (int(laserCloudCornerLast->points[j].intensity) > closestPointScan + 2.5) {
                            break;
                        }

                        pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) * 
                                     (laserCloudCornerLast->points[j].x - pointSel.x) + 
                                     (laserCloudCornerLast->points[j].y - pointSel.y) * 
                                     (laserCloudCornerLast->points[j].y - pointSel.y) + 
                                     (laserCloudCornerLast->points[j].z - pointSel.z) * 
                                     (laserCloudCornerLast->points[j].z - pointSel.z);

                        if (int(laserCloudCornerLast->points[j].intensity) > closestPointScan) {
                            if (pointSqDis < minPointSqDis2) {
                                minPointSqDis2 = pointSqDis;
                                minPointInd2 = j;
                            }
                        }
                    }
                    for (int j = closestPointInd - 1; j >= 0; j--) {
                        if (int(laserCloudCornerLast->points[j].intensity) < closestPointScan - 2.5) {
                            break;
                        }

                        pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) * 
                                     (laserCloudCornerLast->points[j].x - pointSel.x) + 
                                     (laserCloudCornerLast->points[j].y - pointSel.y) * 
                                     (laserCloudCornerLast->points[j].y - pointSel.y) + 
                                     (laserCloudCornerLast->points[j].z - pointSel.z) * 
                                     (laserCloudCornerLast->points[j].z - pointSel.z);

                        if (int(laserCloudCornerLast->points[j].intensity) < closestPointScan) {
                            if (pointSqDis < minPointSqDis2) {
                                minPointSqDis2 = pointSqDis;
                                minPointInd2 = j;
                            }
                        }
                    }
                }

                pointSearchCornerInd1[i] = closestPointInd;
                pointSearchCornerInd2[i] = minPointInd2;
            }

            if (pointSearchCornerInd2[i] >= 0) {

                tripod1 = laserCloudCornerLast->points[pointSearchCornerInd1[i]];
                tripod2 = laserCloudCornerLast->points[pointSearchCornerInd2[i]];

                float x0 = pointSel.x;
                float y0 = pointSel.y;
                float z0 = pointSel.z;
                float x1 = tripod1.x;
                float y1 = tripod1.y;
                float z1 = tripod1.z;
                float x2 = tripod2.x;
                float y2 = tripod2.y;
                float z2 = tripod2.z;

                float m11 = ((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1));
                float m22 = ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1));
                float m33 = ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1));

                float a012 = sqrt(m11 * m11  + m22 * m22 + m33 * m33);

                float l12 = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));

                float la =  ((y1 - y2)*m11 + (z1 - z2)*m22) / a012 / l12;

                float lb = -((x1 - x2)*m11 - (z1 - z2)*m33) / a012 / l12;

                float lc = -((x1 - x2)*m22 + (y1 - y2)*m33) / a012 / l12;

                float ld2 = a012 / l12;

                float s = 1;
                if (iterCount >= 5) {
                    s = 1 - 1.8 * fabs(ld2);
                }

                if (s > 0.1 && ld2 != 0) {
                    coeff.x = s * la; 
                    coeff.y = s * lb;
                    coeff.z = s * lc;
                    coeff.intensity = s * ld2;
                  
                    laserCloudOri->push_back(cornerPointsSharp->points[i]);
                    coeffSel->push_back(coeff);
                }
            }
        }
    }

    void findCorrespondingSurfFeatures(int iterCount){

        int surfPointsFlatNum = surfPointsFlat->points.size();

        for (int i = 0; i < surfPointsFlatNum; i++) {

            TransformToStart(&surfPointsFlat->points[i], &pointSel);

            if (iterCount % 5 == 0) {

                kdtreeSurfLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);
                int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;

                if (pointSearchSqDis[0] < nearestFeatureSearchSqDist) {
                    closestPointInd = pointSearchInd[0];
                    int closestPointScan = int(laserCloudSurfLast->points[closestPointInd].intensity);

                    float pointSqDis, minPointSqDis2 = nearestFeatureSearchSqDist, minPointSqDis3 = nearestFeatureSearchSqDist;
                    for (int j = closestPointInd + 1; j < surfPointsFlatNum; j++) {
                        if (int(laserCloudSurfLast->points[j].intensity) > closestPointScan + 2.5) {
                            break;
                        }

                        pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) * 
                                     (laserCloudSurfLast->points[j].x - pointSel.x) + 
                                     (laserCloudSurfLast->points[j].y - pointSel.y) * 
                                     (laserCloudSurfLast->points[j].y - pointSel.y) + 
                                     (laserCloudSurfLast->points[j].z - pointSel.z) * 
                                     (laserCloudSurfLast->points[j].z - pointSel.z);

                        if (int(laserCloudSurfLast->points[j].intensity) <= closestPointScan) {
                            if (pointSqDis < minPointSqDis2) {
                              minPointSqDis2 = pointSqDis;
                              minPointInd2 = j;
                            }
                        } else {
                            if (pointSqDis < minPointSqDis3) {
                                minPointSqDis3 = pointSqDis;
                                minPointInd3 = j;
                            }
                        }
                    }
                    for (int j = closestPointInd - 1; j >= 0; j--) {
                        if (int(laserCloudSurfLast->points[j].intensity) < closestPointScan - 2.5) {
                            break;
                        }

                        pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) * 
                                     (laserCloudSurfLast->points[j].x - pointSel.x) + 
                                     (laserCloudSurfLast->points[j].y - pointSel.y) * 
                                     (laserCloudSurfLast->points[j].y - pointSel.y) + 
                                     (laserCloudSurfLast->points[j].z - pointSel.z) * 
                                     (laserCloudSurfLast->points[j].z - pointSel.z);

                        if (int(laserCloudSurfLast->points[j].intensity) >= closestPointScan) {
                            if (pointSqDis < minPointSqDis2) {
                                minPointSqDis2 = pointSqDis;
                                minPointInd2 = j;
                            }
                        } else {
                            if (pointSqDis < minPointSqDis3) {
                                minPointSqDis3 = pointSqDis;
                                minPointInd3 = j;
                            }
                        }
                    }
                }

                pointSearchSurfInd1[i] = closestPointInd;
                pointSearchSurfInd2[i] = minPointInd2;
                pointSearchSurfInd3[i] = minPointInd3;
            }

            if (pointSearchSurfInd2[i] >= 0 && pointSearchSurfInd3[i] >= 0) {

                tripod1 = laserCloudSurfLast->points[pointSearchSurfInd1[i]];
                tripod2 = laserCloudSurfLast->points[pointSearchSurfInd2[i]];
                tripod3 = laserCloudSurfLast->points[pointSearchSurfInd3[i]];

                float pa = (tripod2.y - tripod1.y) * (tripod3.z - tripod1.z) 
                         - (tripod3.y - tripod1.y) * (tripod2.z - tripod1.z);
                float pb = (tripod2.z - tripod1.z) * (tripod3.x - tripod1.x) 
                         - (tripod3.z - tripod1.z) * (tripod2.x - tripod1.x);
                float pc = (tripod2.x - tripod1.x) * (tripod3.y - tripod1.y) 
                         - (tripod3.x - tripod1.x) * (tripod2.y - tripod1.y);
                float pd = -(pa * tripod1.x + pb * tripod1.y + pc * tripod1.z);

                float ps = sqrt(pa * pa + pb * pb + pc * pc);

                pa /= ps;
                pb /= ps;
                pc /= ps;
                pd /= ps;

                float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

                float s = 1;
                if (iterCount >= 5) {
                    s = 1 - 1.8 * fabs(pd2) / sqrt(sqrt(pointSel.x * pointSel.x
                            + pointSel.y * pointSel.y + pointSel.z * pointSel.z));
                }

                if (s > 0.1 && pd2 != 0) {
                    coeff.x = s * pa;
                    coeff.y = s * pb;
                    coeff.z = s * pc;
                    coeff.intensity = s * pd2;

                    laserCloudOri->push_back(surfPointsFlat->points[i]);
                    coeffSel->push_back(coeff);
                }
            }
        }
    }

    bool solveTransformationNumerically(const std::vector<int>& variableIndices,
                                        int iterCount)
    {
        const int pointSelNum = static_cast<int>(laserCloudOri->points.size());
        const int variableCount = static_cast<int>(variableIndices.size());
        cv::Mat matA(pointSelNum, variableCount, CV_32F, cv::Scalar::all(0));
        cv::Mat matB(pointSelNum, 1, CV_32F, cv::Scalar::all(0));

        for (int i = 0; i < pointSelNum; ++i) {
            const PointType& point = laserCloudOri->points[i];
            const PointType& coefficient = coeffSel->points[i];

            for (int j = 0; j < variableCount; ++j) {
                const int variableIndex = variableIndices[j];
                const float epsilon = variableIndex < 3 ? 1e-4f : 1e-3f;
                float transformPlus[6];
                float transformMinus[6];
                std::copy(transformCur, transformCur + 6, transformPlus);
                std::copy(transformCur, transformCur + 6, transformMinus);
                transformPlus[variableIndex] += epsilon;
                transformMinus[variableIndex] -= epsilon;

                const Eigen::Vector3f pointPlus = transformPointToStart(point, transformPlus);
                const Eigen::Vector3f pointMinus = transformPointToStart(point, transformMinus);
                const Eigen::Vector3f derivative =
                    (pointPlus - pointMinus) / (2.0f * epsilon);
                matA.at<float>(i, j) = coefficient.x * derivative.x() +
                                       coefficient.y * derivative.y() +
                                       coefficient.z * derivative.z();
            }
            matB.at<float>(i, 0) = -0.05f * coefficient.intensity;
        }

        cv::Mat matAt;
        cv::Mat matAtA;
        cv::Mat matAtB;
        cv::Mat matX;
        cv::transpose(matA, matAt);
        matAtA = matAt * matA;
        matAtB = matAt * matB;
        cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

        if (iterCount == 0) {
            cv::Mat eigenValues;
            cv::Mat eigenVectors;
            cv::eigen(matAtA, eigenValues, eigenVectors);
            cv::Mat retainedEigenvectors = eigenVectors.clone();
            isDegenerate = false;
            for (int i = variableCount - 1; i >= 0; --i) {
                if (eigenValues.at<float>(0, i) < 10.0f) {
                    retainedEigenvectors.row(i).setTo(0);
                    isDegenerate = true;
                } else {
                    break;
                }
            }
            matP = eigenVectors.inv() * retainedEigenvectors;
        }

        if (isDegenerate)
            matX = matP * matX;

        float deltaRotationDegSq = 0.0f;
        float deltaTranslationCmSq = 0.0f;
        for (int j = 0; j < variableCount; ++j) {
            float increment = matX.at<float>(j, 0);
            if (!std::isfinite(increment))
                increment = 0.0f;
            const int variableIndex = variableIndices[j];
            transformCur[variableIndex] += increment;
            if (variableIndex < 3)
                deltaRotationDegSq += std::pow(rad2deg(increment), 2);
            else
                deltaTranslationCmSq += std::pow(increment * 100.0f, 2);
        }

        return std::sqrt(deltaRotationDegSq) >= 0.1f ||
               std::sqrt(deltaTranslationCmSq) >= 0.1f;
    }

    bool calculateTransformationSurf(int iterCount)
    {
        // Ground/surface constraints primarily observe roll, pitch and z in FLU.
        return solveTransformationNumerically({0, 1, 5}, iterCount);
    }

    bool calculateTransformationCorner(int iterCount)
    {
        // Edge constraints primarily observe yaw and horizontal translation in FLU.
        return solveTransformationNumerically({2, 3, 4}, iterCount);
    }

    bool calculateTransformation(int iterCount)
    {
        return solveTransformationNumerically({0, 1, 2, 3, 4, 5}, iterCount);
    }

    void checkSystemInitialization(){

        pcl::PointCloud<PointType>::Ptr laserCloudTemp = cornerPointsLessSharp;
        cornerPointsLessSharp = laserCloudCornerLast;
        laserCloudCornerLast = laserCloudTemp;

        laserCloudTemp = surfPointsLessFlat;
        surfPointsLessFlat = laserCloudSurfLast;
        laserCloudSurfLast = laserCloudTemp;

        kdtreeCornerLast->setInputCloud(laserCloudCornerLast);
        kdtreeSurfLast->setInputCloud(laserCloudSurfLast);

        laserCloudCornerLastNum = laserCloudCornerLast->points.size();
        laserCloudSurfLastNum = laserCloudSurfLast->points.size();

        sensor_msgs::PointCloud2 laserCloudCornerLast2;
        pcl::toROSMsg(*laserCloudCornerLast, laserCloudCornerLast2);
        laserCloudCornerLast2.header.stamp = cloudHeader.stamp;
        laserCloudCornerLast2.header.frame_id = "base_link";
        pubLaserCloudCornerLast.publish(laserCloudCornerLast2);

        sensor_msgs::PointCloud2 laserCloudSurfLast2;
        pcl::toROSMsg(*laserCloudSurfLast, laserCloudSurfLast2);
        laserCloudSurfLast2.header.stamp = cloudHeader.stamp;
        laserCloudSurfLast2.header.frame_id = "base_link";
        pubLaserCloudSurfLast.publish(laserCloudSurfLast2);

        transformSum[0] += imuRollStart;
        transformSum[1] += imuPitchStart;

        systemInitedLM = true;
    }

    void updateInitialGuess(){

        imuPitchLast = imuPitchCur;
        imuYawLast = imuYawCur;
        imuRollLast = imuRollCur;

        imuShiftFromStartX = imuShiftFromStartXCur;
        imuShiftFromStartY = imuShiftFromStartYCur;
        imuShiftFromStartZ = imuShiftFromStartZCur;

        imuVeloFromStartX = imuVeloFromStartXCur;
        imuVeloFromStartY = imuVeloFromStartYCur;
        imuVeloFromStartZ = imuVeloFromStartZCur;

        if (imuAngularFromStartX != 0 || imuAngularFromStartY != 0 || imuAngularFromStartZ != 0){
            transformCur[0] = -imuAngularFromStartX;
            transformCur[1] = -imuAngularFromStartY;
            transformCur[2] = -imuAngularFromStartZ;
        }
        
        if (imuVeloFromStartX != 0 || imuVeloFromStartY != 0 || imuVeloFromStartZ != 0){
            transformCur[3] -= imuVeloFromStartX * scanPeriod;
            transformCur[4] -= imuVeloFromStartY * scanPeriod;
            transformCur[5] -= imuVeloFromStartZ * scanPeriod;
        }
    }

    void updateTransformation(){

        if (laserCloudCornerLastNum < 10 || laserCloudSurfLastNum < 100)
            return;

        for (int iterCount1 = 0; iterCount1 < 25; iterCount1++) {
            laserCloudOri->clear();
            coeffSel->clear();

            findCorrespondingSurfFeatures(iterCount1);

            if (laserCloudOri->points.size() < 10)
                continue;
            if (calculateTransformationSurf(iterCount1) == false)
                break;
        }

        for (int iterCount2 = 0; iterCount2 < 25; iterCount2++) {

            laserCloudOri->clear();
            coeffSel->clear();

            findCorrespondingCornerFeatures(iterCount2);

            if (laserCloudOri->points.size() < 10)
                continue;
            if (calculateTransformationCorner(iterCount2) == false)
                break;
        }
    }

    void integrateTransformation()
    {
        const Eigen::Matrix3f rotationSum = rotationMatrix(
            transformSum[0], transformSum[1], transformSum[2]);
        const Eigen::Matrix3f rotationCur = rotationMatrix(
            transformCur[0], transformCur[1], transformCur[2]);
        const Eigen::Matrix3f rotationNew = rotationSum * rotationCur.transpose();

        const Eigen::Vector3f translationSum(
            transformSum[3], transformSum[4], transformSum[5]);
        const Eigen::Vector3f translationCur(
            transformCur[3] - imuShiftFromStartX,
            transformCur[4] - imuShiftFromStartY,
            transformCur[5] - imuShiftFromStartZ);
        const Eigen::Vector3f translationNew =
            translationSum - rotationNew * translationCur;

        float roll, pitch, yaw;
        matrixToRPY(rotationNew, roll, pitch, yaw);
        PluginIMURotation(roll, pitch, yaw,
                          imuRollStart, imuPitchStart, imuYawStart,
                          imuRollLast, imuPitchLast, imuYawLast,
                          roll, pitch, yaw);

        transformSum[0] = roll;
        transformSum[1] = pitch;
        transformSum[2] = yaw;
        transformSum[3] = translationNew.x();
        transformSum[4] = translationNew.y();
        transformSum[5] = translationNew.z();
    }

    void publishOdometry()
    {
        const geometry_msgs::Quaternion orientation =
            tf::createQuaternionMsgFromRollPitchYaw(
                transformSum[0], transformSum[1], transformSum[2]);

        laserOdometry.header.stamp = cloudHeader.stamp;
        laserOdometry.pose.pose.orientation = orientation;
        laserOdometry.pose.pose.position.x = transformSum[3];
        laserOdometry.pose.pose.position.y = transformSum[4];
        laserOdometry.pose.pose.position.z = transformSum[5];
        pubLaserOdometry.publish(laserOdometry);

        laserOdometryTrans.stamp_ = cloudHeader.stamp;
        laserOdometryTrans.setRotation(tf::Quaternion(
            orientation.x, orientation.y, orientation.z, orientation.w));
        laserOdometryTrans.setOrigin(tf::Vector3(
            transformSum[3], transformSum[4], transformSum[5]));
        tfBroadcaster.sendTransform(laserOdometryTrans);
    }

    void adjustOutlierCloud()
    {
        // imageProjection already publishes outliers in FLU coordinates.
        // Intentionally no coordinate permutation here.
    }

    void publishCloudsLast(){

        updateImuRollPitchYawStartSinCos();

        int cornerPointsLessSharpNum = cornerPointsLessSharp->points.size();
        for (int i = 0; i < cornerPointsLessSharpNum; i++) {
            TransformToEnd(&cornerPointsLessSharp->points[i], &cornerPointsLessSharp->points[i]);
        }


        int surfPointsLessFlatNum = surfPointsLessFlat->points.size();
        for (int i = 0; i < surfPointsLessFlatNum; i++) {
            TransformToEnd(&surfPointsLessFlat->points[i], &surfPointsLessFlat->points[i]);
        }

        pcl::PointCloud<PointType>::Ptr laserCloudTemp = cornerPointsLessSharp;
        cornerPointsLessSharp = laserCloudCornerLast;
        laserCloudCornerLast = laserCloudTemp;

        laserCloudTemp = surfPointsLessFlat;
        surfPointsLessFlat = laserCloudSurfLast;
        laserCloudSurfLast = laserCloudTemp;

        laserCloudCornerLastNum = laserCloudCornerLast->points.size();
        laserCloudSurfLastNum = laserCloudSurfLast->points.size();

        if (laserCloudCornerLastNum > 10 && laserCloudSurfLastNum > 100) {
            kdtreeCornerLast->setInputCloud(laserCloudCornerLast);
            kdtreeSurfLast->setInputCloud(laserCloudSurfLast);
        }

        frameCount++;

        if (frameCount >= skipFrameNum + 1) {

            frameCount = 0;

            adjustOutlierCloud();
            sensor_msgs::PointCloud2 outlierCloudLast2;
            pcl::toROSMsg(*outlierCloud, outlierCloudLast2);
            outlierCloudLast2.header.stamp = cloudHeader.stamp;
            outlierCloudLast2.header.frame_id = "base_link";
            pubOutlierCloudLast.publish(outlierCloudLast2);

            sensor_msgs::PointCloud2 laserCloudCornerLast2;
            pcl::toROSMsg(*laserCloudCornerLast, laserCloudCornerLast2);
            laserCloudCornerLast2.header.stamp = cloudHeader.stamp;
            laserCloudCornerLast2.header.frame_id = "base_link";
            pubLaserCloudCornerLast.publish(laserCloudCornerLast2);

            sensor_msgs::PointCloud2 laserCloudSurfLast2;
            pcl::toROSMsg(*laserCloudSurfLast, laserCloudSurfLast2);
            laserCloudSurfLast2.header.stamp = cloudHeader.stamp;
            laserCloudSurfLast2.header.frame_id = "base_link";
            pubLaserCloudSurfLast.publish(laserCloudSurfLast2);
        }
    }

    void runFeatureAssociation()
    {

        if (newSegmentedCloud && newSegmentedCloudInfo && newOutlierCloud &&
            std::abs(timeNewSegmentedCloudInfo - timeNewSegmentedCloud) < 0.05 &&
            std::abs(timeNewOutlierCloud - timeNewSegmentedCloud) < 0.05){

            newSegmentedCloud = false;
            newSegmentedCloudInfo = false;
            newOutlierCloud = false;
        }else{
            return;
        }
        /**
        	1. Feature Extraction
        */
        adjustDistortion();

        calculateSmoothness();

        markOccludedPoints(); // 剔除遮挡点/平行光束点，防止被提取为特征

        extractFeatures();

        publishCloud(); // cloud for visualization
	
        /**
		2. Feature Association
        */
        if (!systemInitedLM) {
            checkSystemInitialization();
            return;
        }

        updateInitialGuess();

        updateTransformation();

        integrateTransformation();

        publishOdometry();

        publishCloudsLast(); // cloud to mapOptimization
    }
};




int main(int argc, char** argv)
{
    ros::init(argc, argv, "lego_loam");

    ROS_INFO("\033[1;32m---->\033[0m Feature Association Started.");

    FeatureAssociation FA;

    ros::Rate rate(200);
    while (ros::ok())
    {
        ros::spinOnce();

        FA.runFeatureAssociation();

        rate.sleep();
    }
    
    ros::spin();
    return 0;
}
