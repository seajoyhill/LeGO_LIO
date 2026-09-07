#ifndef LEGO_CALIB_COMMON_LIB_H
#define LEGO_CALIB_COMMON_LIB_H
#include <so3_math.h>
#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <color.h>
#include <scope_timer.hpp>
#define PBSTR "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||"
#define PBWIDTH 30
#define DIM_STATE (23)
#define VEC_FROM_ARRAY(val) val[0], val[1], val[2]
#define MAT_FROM_ARRAY(val) val[0], val[1], val[2], val[3], val[4], val[5], val[6], val[7], val[8]
using namespace std; using namespace Eigen;
#define VD(a) Eigen::Matrix<double, a, 1>
#define G_m_s2 9.81
#define Eye3d (Eigen::Matrix3d::Identity())
#define Zero3d (Eigen::Vector3d::Zero())
#endif
