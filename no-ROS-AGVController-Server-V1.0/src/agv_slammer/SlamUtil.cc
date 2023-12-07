/*
 * @Author: yang.cheng yang.cheng@bgirobot.com
 * @Date: 2023-07-27 08:13:55
 * @LastEditors: yang.cheng yang.cheng@bgirobot.com
 * @LastEditTime: 2023-08-08 16:53:40
 * @FilePath: /agv_controller/src/agv_slammer/SlamUtil.cc
 * @Description: SLAM 数据类型转换相关方法定义
 */

#include "time.h"
#include "glog/logging.h"
#include "cartographer/common/time.h"
#include "cartographer/transform/transform.h"
#include "SlamUtil.h"

/**
 * @brief 解析用于定位和建图功能的传感器配置
 * @param topic 传感器类(scan)
 * @param num_topics 传感器数量
 * @return 该类传感器的所有话题 (scan_1 scan_2...)等
 */
std::vector<std::string> ComputeRepeatedTopicNames(const std::string& topic,
                                                   const int num_topics) {
  CHECK_GE(num_topics, 0);
  if (num_topics == 1) {
    return {topic};
  }
  std::vector<std::string> topics;
  topics.reserve(num_topics);
  for (int i = 0; i < num_topics; ++i) {
    topics.emplace_back(topic + "_" + std::to_string(i + 1));
  }
  return topics;
}

/**
 * @brief 换算Linux系统时间到Cartographer管理的时间
 * @param time 标准Linux系统时间（标准时间类型）
 * @return cartographer管理的时间
 */
::cartographer::common::Time FromSysTime(TAgvTimeStamp time) {
  // The epoch of the ICU Universal Time Scale is "0001-01-01 00:00:00.0 +0000",
  // exactly 719162 days before the Unix epoch.
  return ::cartographer::common::FromUniversal(
      (time.sec +
       ::cartographer::common::kUtsEpochOffsetFromUnixEpochInSeconds) *
          10000000ll +
      (time.nsec + 50) / 100);  // + 50 to get the rounding correct.
}


/**
 * @name: 辅助的位姿相关数据结构转换方法
 * @des: 
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
using ::cartographer::transform::Rigid3d;
using ::cartographer::transform::Rigid2d;
using ::cartographer::transform::Embed3D;
using ::cartographer::transform::GetYaw;
//Transform AgvPose3D to Rigid3d
Rigid3d ToRigid3d(const TAgvPose3D& pose)
{
    return Rigid3d({pose.position.x, pose.position.y, pose.position.z},
                  ToEigen(pose.orientation));
}

Rigid3d ToRigid3d(const TAgvPose2D& pose)
{
    Rigid2d r2Trans({pose.x, pose.y}, pose.phi);
    return Embed3D(r2Trans);
}

Eigen::Vector3d ToEigen(const TAgvVector3D& vector3) {
    return Eigen::Vector3d(vector3.x, vector3.y, vector3.z);
}

Eigen::Quaterniond ToEigen(const TAgvOrientation& quaternion) {
    return Eigen::Quaterniond(quaternion.w, quaternion.x, quaternion.y,
                            quaternion.z);
}
//系统时间转换成传感器时间戳
 TAgvTimeStamp fromNSec(int64_t t)
  {
    TAgvTimeStamp tAgvTimeStamp;
    int64_t sec64 = t / 1000000000LL;
    if (sec64 < std::numeric_limits<int32_t>::min() || sec64 > std::numeric_limits<int32_t>::max())
      throw std::runtime_error("Duration is out of dual 32-bit range");
    tAgvTimeStamp.sec = static_cast<long int>(sec64);
    tAgvTimeStamp.nsec = static_cast<long int>(t % 1000000000LL);
    
    return tAgvTimeStamp;
  }

/**
 * @name: 
 * @des: 
 * @param {Rigid3d&} rigid3d
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
TAgvPose3D ToAgvPoseMsgPose(const Rigid3d& rigid3d)
{
    TAgvPose3D pose;
    pose.position = ToAgvMsgPoint(rigid3d.translation());
    pose.orientation.w = rigid3d.rotation().w();
    pose.orientation.x = rigid3d.rotation().x();
    pose.orientation.y = rigid3d.rotation().y();
    pose.orientation.z = rigid3d.rotation().z();
    return pose;
}

/**
 * @name: 
 * @des: 
 * @param {Vector3d&} vector3d
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
TAgvPosition3D ToAgvMsgPoint(const Eigen::Vector3d& vector3d)
 {
    TAgvPosition3D point;
    point.x = vector3d.x();
    point.y = vector3d.y();
    point.z = vector3d.z();
    return point;
 }

 /**
  * @name: 
  * @des: 
  * @param {Rigid3d&} r3
  * @return {*}
  * @author: yang.cheng
  * @ver: 1.01
  */
TAgvPose2D ToAgvPose2D(const Rigid3d& r3)
{
  TAgvPose2D tPose2D;
  Rigid2d r2;
  tPose2D.x = r3.translation().x();
  tPose2D.y = r3.translation().y();
  tPose2D.phi = GetYaw(r3.rotation());
  return tPose2D;
}
/**
 * @name: AngleNormalize
 * @des:  正则化角度
 * @param {double} z
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
double AngleNormalize(double z)
{
  return atan2(sin(z),cos(z));
}

/**
 * @name: AngleDiff
 * @des:  正则化角度差
 * @param {double} a
 * @param {double} b
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
double AngleDiff(double a, double b)
{
  double d1, d2;
  a = AngleNormalize(a);
  b = AngleNormalize(b);
  d1 = a-b;
  d2 = 2*M_PI - fabs(d1);
  if(d1 > 0)
    d2 *= -1.0;
  if(fabs(d1) < fabs(d2))
    return(d1);
  else
    return(d2);
}