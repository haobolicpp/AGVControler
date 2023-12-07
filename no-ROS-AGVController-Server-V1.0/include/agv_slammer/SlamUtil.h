/*
 * @Author: yang.cheng yang.cheng@bgirobot.com
 * @Date: 2023-04-10 08:32:38
 * @LastEditors: yang.cheng yang.cheng@bgirobot.com
 * @LastEditTime: 2023-08-08 16:43:19
 * @FilePath: /agv_controller/include/agv_slammer/SlamUtil.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#ifndef SLAMUTIL_H
#define SLAMUTIL_H

#include <string>
#include <vector>
#include <cmath>
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "agv_type.h"

#include "cartographer/transform/rigid_transform.h"
#include "cartographer/common/time.h"

// Default topic names; expected to be remapped as needed.
constexpr char kLaserScanTopic[] = "scan";
constexpr char kMultiEchoLaserScanTopic[] = "echoes";
constexpr char kPointCloud2Topic[] = "points2";
constexpr char kImuTopic[] = "imu";
constexpr char kOdometryTopic[] = "odom";
constexpr char kNavSatFixTopic[] = "fix";
constexpr char kLandmarkTopic[] = "landmark";
constexpr char kFinishTrajectoryServiceName[] = "finish_trajectory";
constexpr char kOccupancyGridTopic[] = "map";
constexpr char kScanMatchedPointCloudTopic[] = "scan_matched_points2";
constexpr char kSubmapListTopic[] = "submap_list";
constexpr char kTrackedPoseTopic[] = "tracked_pose";
constexpr char kSubmapQueryServiceName[] = "submap_query";
constexpr char kTrajectoryQueryServiceName[] = "trajectory_query";
constexpr char kStartTrajectoryServiceName[] = "start_trajectory";
constexpr char kWriteStateServiceName[] = "write_state";
constexpr char kGetTrajectoryStatesServiceName[] = "get_trajectory_states";
constexpr char kReadMetricsServiceName[] = "read_metrics";
constexpr char kTrajectoryNodeListTopic[] = "trajectory_node_list";
constexpr char kLandmarkPosesListTopic[] = "landmark_poses_list";
constexpr char kConstraintListTopic[] = "constraint_list";
constexpr double kConstraintPublishPeriodSec = 0.5;
constexpr double kTopicMismatchCheckDelaySec = 3.0;

constexpr int kInfiniteSubscriberQueueSize = 0;
constexpr int kLatestOnlyPublisherQueueSize = 1;

// For multiple topics adds numbers to the topic name and returns the list.
std::vector<std::string> ComputeRepeatedTopicNames(const std::string& topic,
                                                   int num_topics);


// system time(1970) to cartographer time(0001-01-01)
::cartographer::common::Time FromSysTime(TAgvTimeStamp time);


//-------------------------数据转换定义------------------------
using ::cartographer::transform::Rigid3d;
using ::cartographer::transform::Rigid2d;

Rigid3d ToRigid3d(const TAgvPose3D& pose);
Rigid3d ToRigid3d(const TAgvPose2D& pose);
Eigen::Vector3d ToEigen(const TAgvVector3D& vector3);
Eigen::Quaterniond ToEigen(const TAgvOrientation& quaternion);

TAgvTimeStamp fromNSec(int64_t t);
TAgvPose3D ToAgvPoseMsgPose(const Rigid3d& rigid3d);
TAgvPosition3D ToAgvMsgPoint(const Eigen::Vector3d& vector3d);
TAgvPose2D ToAgvPose2D(const Rigid3d& r3);

double AngleNormalize(double z);
double AngleDiff(double a, double b);
#endif // SLAMUTIL_H