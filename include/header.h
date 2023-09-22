#ifndef CKYLOAM_HEADER_H
#define CKYLOAM_HEADER_H

#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <pcl/filters/voxel_grid.h>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/kdtree/kdtree_flann.h"
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <pcl/registration/icp.h>

#include<chrono>
#include<ctime>
//sophus
#include <sophus/so3.hpp>
#include <sophus/se3.hpp>

//g2o
#include<g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <fstream>
//int N_SCAN_ROW=16;    // 激光雷达线数，16线



int N_SCAN_ROW=64;      // 激光雷达线数，64线

/*
自动求导标志*/
bool autograde=true;
bool local_params =false;

/**
 * 点信息结构
 * value,indexInRow
 */
typedef struct {
    float value;        // 属性值，比如曲率
    int indexInRow;     // 行内点索引
} PointInfo;

/**
 * 点结构
 * x,y,z,intensity,ring,time
 * 按照pcl规定，自定义点云结构，主要用于点云数据的接收与解析
 * 自定义点云结构的缺点是，在有些pcl函数中无法识别，比如kdtree中
*/
struct VelodynePointXYZIRT {
    PCL_ADD_POINT4D       // 位置
    PCL_ADD_INTENSITY;    // 激光点反射强度，也可以存点的索引
    //uint16_t ring;      // 各点所在扫描线号，可选
    //float time;         // 各点时间戳，相对于帧第一个点时间差，可选
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;          // 内存16字节对齐，EIGEN SSE优化要求
// 注册为PCL点云格式
POINT_CLOUD_REGISTER_POINT_STRUCT (VelodynePointXYZIRT,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
    //(uint16_t, ring, ring)(float, time, time)
)

/**
 * 6D位姿点结构定义(x,y,z,roll,pitch,yaw)
 * x,y,z,intensity,roll,pitch,yaw,time
*/
struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
    (float, roll, roll)(float, pitch, pitch)(float, yaw, yaw)
    (double, time, time)
)

#endif
