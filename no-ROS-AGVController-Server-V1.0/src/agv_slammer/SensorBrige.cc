/*********************************************************************
 * AgvSlammer Class Definiation
 * 主要实现与Cartogrpher的调用接口
 * Copyright: BGI
 * Author: yang.cheng
 *********************************************************************/

#include "absl/memory/memory.h"

#include "agv_type.h"
#include "AgvTf.h"

#include "SensorBrige.h"
#include "SensorConvert.h"
#include "SlamUtil.h"


namespace carto = ::cartographer;

namespace {
/**
 * @brief 传感器话题合法性检验
 * @param 
 * @return
 */
const std::string& CheckNoLeadingSlash(const std::string& frame_id) {
  if (frame_id.size() > 0) {
    CHECK_NE(frame_id[0], '/') << "The frame_id " << frame_id
                               << " should not start with a /. See 1.7 in "
                                  "http://wiki.ros.org/tf2/Migration.";
  }
  return frame_id;
}

}  // namespace

/**
 * @brief SensorBridge构造
 * @param 
 * @return
 */
SensorBridge::SensorBridge(
    const int num_subdivisions_per_laser_scan,
    const std::string& tracking_frame,
    const double lookup_transform_timeout_sec, AgvTf *poAgvTf,
    carto::mapping::TrajectoryBuilderInterface* const trajectory_builder)
    : num_subdivisions_per_laser_scan_(num_subdivisions_per_laser_scan),
      moAgvTf_(poAgvTf), tracking_frame_(tracking_frame),
      trajectory_builder_(trajectory_builder) {}


/**
 * @brief 自定义Odom数据类型转换到Carto的Odom标准数据类型
 * @param msg 自定义Odom数据类型
 * @return unique_ptr<OdometryData>
 */
std::unique_ptr<carto::sensor::OdometryData> SensorBridge::ToOdometryData(const TAgvOdomMsg *msg)
{
    const carto::common::Time time = FromSysTime(msg->header.stamp);

    return absl::make_unique<carto::sensor::OdometryData>
        (carto::sensor::OdometryData{time, ToRigid3d(msg->pose)});
}


/**
 * @brief 自定义Odom数据类型转换到Carto的Odom标准数据类型
 * @param msg 自定义Odom数据类型
 * @return unique_ptr<OdometryData>
 */
void SensorBridge::HandleOdometryMessage(const std::string& sensor_id, const TAgvOdomMsg *msg)
{
  std::unique_ptr<carto::sensor::OdometryData> odometry_data = ToOdometryData(msg);
  if (odometry_data != nullptr)
  {
    trajectory_builder_->AddSensorData(sensor_id,
        carto::sensor::OdometryData{odometry_data->time, odometry_data->pose});
  }
}

/**
 * @brief 自定义Imu数据类型转换到Carto的Imu标准数据类型
 * @param msg 自定义IMU数据类型
 * @return unique_ptr<ImuData>
 */
std::unique_ptr<::cartographer::sensor::ImuData> SensorBridge::ToImuData(const TAgvImuMsg *msg)
{   
    const carto::common::Time time = FromSysTime(msg->header.stamp);
    //todo 合法性检验+位姿变换
    return absl::make_unique<carto::sensor::ImuData>(carto::sensor::ImuData{
    time, ToEigen(msg->linear_acceleration), ToEigen(msg->angular_velocity)});
}

/**
 * @brief 自定义Imu数据类型转换到Carto的Imu标准数据类型
 * @param msg 自定义Imu数据类型
 * @return 
 */
void SensorBridge::HandleImuMessage(const std::string& sensor_id, const TAgvImuMsg *msg)
{
    std::unique_ptr<carto::sensor::ImuData> imu_data = ToImuData(msg);
    if (imu_data != nullptr)
    {
        trajectory_builder_->AddSensorData(sensor_id,
        carto::sensor::ImuData{imu_data->time, imu_data->linear_acceleration,
                                imu_data->angular_velocity});
    }
}

/**
 * @brief 自定义激光雷达数据类型转换到Carto的标准点云数据
 * @param msg 自定义Laser数据类型
 * @return 
 */
void SensorBridge::HandleLaserScanMessage(const std::string& sensor_id, const TAgvLaserScanMsg *msg)
{
    carto::sensor::PointCloudWithIntensities point_cloud;
    carto::common::Time time;
    std::tie(point_cloud, time) = ToPointCloudWithIntensities(*msg);
    HandleLaserScan(sensor_id, time, msg->header.frame_id, point_cloud);
}


/**
 * @brief 激光雷达数据点云分割
 * @param 
 * @return 
 */
void SensorBridge::HandleLaserScan(
    const std::string& sensor_id, const carto::common::Time time,
    const std::string& frame_id,
    const carto::sensor::PointCloudWithIntensities& points) {
  if (points.points.empty()) {
    return;
  }
  CHECK_LE(points.points.back().time, 0.f);
  // TODO(gaschler): Use per-point time instead of subdivisions.
  for (int i = 0; i != num_subdivisions_per_laser_scan_; ++i) {
    const size_t start_index =
        points.points.size() * i / num_subdivisions_per_laser_scan_;
    const size_t end_index =
        points.points.size() * (i + 1) / num_subdivisions_per_laser_scan_;
    carto::sensor::TimedPointCloud subdivision(
        points.points.begin() + start_index, points.points.begin() + end_index);
    if (start_index == end_index) {
      continue;
    }
    const double time_to_subdivision_end = subdivision.back().time;
    // `subdivision_time` is the end of the measurement so sensor::Collator will
    // send all other sensor data first.
    const carto::common::Time subdivision_time =
        time + carto::common::FromSeconds(time_to_subdivision_end);
    auto it = sensor_to_previous_subdivision_time_.find(sensor_id);
    if (it != sensor_to_previous_subdivision_time_.end() &&
        it->second >= subdivision_time) {
      LOG(WARNING) << "Ignored subdivision of a LaserScan message from sensor "
                   << sensor_id << " because previous subdivision time "
                   << it->second << " is not before current subdivision time "
                   << subdivision_time;
      continue;
    }
    sensor_to_previous_subdivision_time_[sensor_id] = subdivision_time;
    for (auto& point : subdivision) {
      point.time -= time_to_subdivision_end;
    }
    CHECK_EQ(subdivision.back().time, 0.f);
    HandleRangefinder(sensor_id, subdivision_time, frame_id, subdivision);
  }
}

/**
 * @brief 自定义激光雷达数据类型转换到Carto的标准点云数据
 * @param 
 * @return
 */
void SensorBridge::HandleRangefinder(
    const std::string& sensor_id, const carto::common::Time time,
    const std::string& frame_id, const carto::sensor::TimedPointCloud& ranges) {
  if (!ranges.empty()) {
    CHECK_LE(ranges.back().time, 0.f);
  }

  int isFailed;
  Rigid3d sensor_to_tracking; //From system Tf
  isFailed = moAgvTf_->GetTransform(tracking_frame_, frame_id, sensor_to_tracking);

  //To Carto transform 
  carto::transform::Rigid3d carto_sensor_to_tracking 
          = carto::transform::Rigid3d(sensor_to_tracking.translation(), 
                        sensor_to_tracking.rotation());
  //const auto sensor_to_tracking =
  //    tf_bridge_.LookupToTracking(time, CheckNoLeadingSlash(frame_id));

  if (0 == isFailed)
  {
    trajectory_builder_->AddSensorData(
        sensor_id, carto::sensor::TimedPointCloudData{
                       time, sensor_to_tracking.translation().cast<float>(),
                       carto::sensor::TransformTimedPointCloud(
                           ranges, carto_sensor_to_tracking.cast<float>())});
  }
}