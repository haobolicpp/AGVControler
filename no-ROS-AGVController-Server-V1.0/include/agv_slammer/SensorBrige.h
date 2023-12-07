/*
 * @Author: yang.cheng yang.cheng@bgirobot.com
 * @Date: 2023-04-10 08:32:38
 * @LastEditors: yang.cheng yang.cheng@bgirobot.com
 * @LastEditTime: 2023-07-25 15:18:01
 * @FilePath: /agv_controller/include/agv_slammer/SensorBrige.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#ifndef SENSOR_BRIDGE_H
#define SENSOR_BRIDGE_H

#include <memory>
#include "agv_type.h"
#include "AgvTf.h"

#include "absl/types/optional.h"
#include "cartographer/mapping/trajectory_builder_interface.h"
#include "cartographer/sensor/imu_data.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"

// Converts Agv Sensor messages into SensorData in tracking frame for the MapBuilder.
class SensorBridge {
public:
    explicit SensorBridge(
    int num_subdivisions_per_laser_scan, const std::string& tracking_frame,
    double lookup_transform_timeout_sec, AgvTf* poAgvTf,
    ::cartographer::mapping::TrajectoryBuilderInterface* trajectory_builder);
    

    SensorBridge(const SensorBridge&) = delete;
    SensorBridge& operator=(const SensorBridge&) = delete;

    //里程计数据转换和处理方法
    std::unique_ptr<::cartographer::sensor::OdometryData> ToOdometryData(const TAgvOdomMsg *msg);
    void HandleOdometryMessage(const std::string& sensor_id,const TAgvOdomMsg *msg);
    
    //惯导数据转换和处理方法
    std::unique_ptr<::cartographer::sensor::ImuData> ToImuData(const TAgvImuMsg *msg);
    void HandleImuMessage(const std::string& sensor_id, const TAgvImuMsg *msg);
    
    //激光雷达数据转换和处理方法
    void HandleLaserScanMessage(const std::string& sensor_id, const TAgvLaserScanMsg *msg);
    void HandlePointCloud2Message(const std::string& sensor_id, const TAgvPointCloud2 *msg);

    //const TfBridge& tf_bridge() const;

private:
    void HandleLaserScan(
        const std::string& sensor_id, ::cartographer::common::Time start_time,
        const std::string& frame_id,
        const ::cartographer::sensor::PointCloudWithIntensities& points);
    void HandleRangefinder(const std::string& sensor_id,
                            ::cartographer::common::Time time,
                            const std::string& frame_id,
                            const ::cartographer::sensor::TimedPointCloud& ranges);

    const int num_subdivisions_per_laser_scan_;
    std::map<std::string, cartographer::common::Time> sensor_to_previous_subdivision_time_;
    //const TfBridge tf_bridge_;
    AgvTf *moAgvTf_;
    const std::string tracking_frame_;

    ::cartographer::mapping::TrajectoryBuilderInterface* const trajectory_builder_;
    absl::optional<::cartographer::transform::Rigid3d> ecef_to_local_frame_;

};


#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_SENSOR_BRIDGE_H
