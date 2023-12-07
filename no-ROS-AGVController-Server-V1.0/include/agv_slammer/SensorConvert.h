/*********************************************************************
 * Sensor 传感器数据采样器和数据转换相关方法定义
 * 主要实现传感器数据的格式转换
 * Copyright: BGI
 * Author: yang.cheng
 *********************************************************************/

#ifndef SENSORCONVERT_H
#define SENSORCONVERT_H

#include "agv_type.h"

#include "cartographer/common/time.h"
#include "cartographer/common/fixed_ratio_sampler.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer/sensor/landmark_data.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/rigid_transform.h"



using FixedRatioSampler = ::cartographer::common::FixedRatioSampler;

struct TrajectorySensorSamplers {
TrajectorySensorSamplers(const double rangefinder_sampling_ratio,
                const double odometry_sampling_ratio,
                const double fixed_frame_pose_sampling_ratio,
                const double imu_sampling_ratio,
                const double landmark_sampling_ratio): 
        rangefinder_sampler(rangefinder_sampling_ratio),
        odometry_sampler(odometry_sampling_ratio),
        fixed_frame_pose_sampler(fixed_frame_pose_sampling_ratio),
        imu_sampler(imu_sampling_ratio),
        landmark_sampler(landmark_sampling_ratio) {}

    FixedRatioSampler rangefinder_sampler;
    FixedRatioSampler odometry_sampler;
    FixedRatioSampler fixed_frame_pose_sampler;
    FixedRatioSampler imu_sampler;
    FixedRatioSampler landmark_sampler;
};

std::tuple<::cartographer::sensor::PointCloudWithIntensities,
    ::cartographer::common::Time>
ToPointCloudWithIntensities(const TAgvLaserScanMsg &msg);

std::tuple<::cartographer::sensor::PointCloudWithIntensities, 
    ::cartographer::common::Time>
LaserScanToPointCloudWithIntensities(const TAgvLaserScanMsg &msg);
//用于传感器数据的序列化和反序列化
int sensorHeaderSerializer(uint8_t *buffer, uint32_t *length, TSensorMsgHeader *msg_raw);
int sensorHeaderDeSerializer(uint8_t *buffer, uint32_t *length, TSensorMsgHeader *msg_raw);
int laserMsgSerializer(uint8_t *buffer, uint32_t *length, TAgvLaserScanMsg *msg_raw);
int laserMsgDeserializer(uint8_t *buffer, uint32_t *length, TAgvLaserScanMsg *msg_raw);
int imuMsgSerializer(uint8_t *buffer, uint32_t *length, TAgvImuMsg *msg_raw);
int imuMsgDeserializer(uint8_t *buffer, uint32_t *length, TAgvImuMsg *msg_raw);
int odomMsgSerializer(uint8_t *buffer, uint32_t *length, TAgvOdomMsg *msg_raw);
int odomMsgDeserializer(uint8_t *buffer, uint32_t *length, TAgvOdomMsg *msg_raw);
int OccupancyGridMsgSerializer(uint8_t *buffer, uint32_t *length, TOccupancyGrid *msg_raw);
int OccupancyGridMsgDeserializer(uint8_t *buffer, uint32_t *length, TOccupancyGrid *msg_raw);
#endif // MSGCOVERT_H