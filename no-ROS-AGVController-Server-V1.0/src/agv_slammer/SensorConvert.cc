/*********************************************************************
 * SensorConert.cc
 * 转换激光雷达数据到SLAM标准格式
 * Copyright: BGI
 * Author: yang.cheng
 *********************************************************************/

#include <string>
#include <cmath>
#include "agv_type.h"
#include "SensorConvert.h"
#include "SlamUtil.h"

#include "cartographer/common/math.h"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer/transform/proto/transform.pb.h"
#include "cartographer/transform/transform.h"
#include "cartographer/sensor/point_cloud.h"



using namespace cartographer::sensor;

/**
 * @brief 兼容多回波雷达数据用
 * @param msg 自定义Laser数据类型
 * @return unique_ptr<ImuData>
 */
bool HasEcho(float)
{ 
    return true; 
}

/**
 * @brief 兼容多回波雷达数据用
 * @param msg 自定义Laser数据类型
 * @return unique_ptr<ImuData>
 */
float GetFirstEcho(float range)
{ 
    return range; 
}

/**
 * @brief 转换LaserScan数据类型到SLAM需求的点云
 * @param msg 自定义Laser数据类型
 * @return unique_ptr<ImuData>
 */
std::tuple<PointCloudWithIntensities, ::cartographer::common::Time>
ToPointCloudWithIntensities(const TAgvLaserScanMsg &msg)
{
    return LaserScanToPointCloudWithIntensities(msg);
}

/**
 * @brief 转换LaserScan数据类型到SLAM需求的点云
 * @param msg 自定义Laser数据类型
 * @return unique_ptr<ImuData>
 */
std::tuple<PointCloudWithIntensities, ::cartographer::common::Time>
LaserScanToPointCloudWithIntensities(const TAgvLaserScanMsg &msg)
{
    //雷达数据的基本参数合法性检验
    CHECK_GE(msg.range_min, 0.f);
    CHECK_GE(msg.range_max, msg.range_min);

    //雷达发布的角度增量合法性检验
    if (msg.angle_increment > 0.f)
    {
        CHECK_GT(msg.angle_max, msg.angle_min);
    } 
    else 
    {
        CHECK_GT(msg.angle_min, msg.angle_max);
    }

    PointCloudWithIntensities point_cloud;
    float angle = msg.angle_min;

    for (size_t i = 0; i < msg.ranges_size; ++i)
    {
        const auto& echoes = msg.ranges[i];
        if (HasEcho(echoes))
        {
            const float first_echo = GetFirstEcho(echoes);

            if (msg.range_min <= first_echo && first_echo <= msg.range_max)
            {
                const Eigen::AngleAxisf rotation(angle, Eigen::Vector3f::UnitZ());
                const cartographer::sensor::TimedRangefinderPoint point{
                rotation * (first_echo * Eigen::Vector3f::UnitX()),
                i * msg.time_increment};
                point_cloud.points.push_back(point);
                // if (msg.intensities.size() > 0)
                if (msg.intensities_size > 0)
                {
                    // CHECK_EQ(msg.intensities.size(), msg.ranges.size());
                    CHECK_EQ(msg.intensities_size, msg.ranges_size);
                    const auto& echo_intensities = msg.intensities[i];
                    CHECK(HasEcho(echo_intensities));
                    point_cloud.intensities.push_back(GetFirstEcho(echo_intensities));
                }
                else 
                {
                    point_cloud.intensities.push_back(0.f);
                }
            }
        }
        angle += msg.angle_increment;
    }//基本点云转换完毕

    //添加SLAM时间戳信息
    ::cartographer::common::Time timestamp = FromSysTime(msg.header.stamp);
    if (!point_cloud.points.empty())
    {
        const double duration = point_cloud.points.back().time;
        timestamp += cartographer::common::FromSeconds(duration);
        for (auto& point : point_cloud.points) 
        {
            point.time -= duration;
        }
    }

    //返回点云和时间戳
    // LOG(INFO) << timestamp;
    return std::make_tuple(point_cloud, timestamp);
}


/**
 * @name: 
 * @des: 
 * @param {uint8_t} *buffer
 * @param {uint32_t} *length
 * @param {TSensorMsgHeader} *msg_raw
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int sensorHeaderSerializer(uint8_t *buffer, uint32_t *length, TSensorMsgHeader *msg_raw)
{
    if (buffer == NULL || length == NULL || msg_raw == NULL)
    {
        return -1;
    }

    memcpy(buffer, msg_raw, sizeof(TSensorMsgHeader));
    return 0;

    // uint32_t offset = 0;
    // uint8_t strBufferLen = 20;//frame_id缓存长度

    // memcpy(buffer, &msg_raw->seq, sizeof(uint32_t));
    // buffer = buffer + sizeof(uint32_t);
    // offset = offset + sizeof(uint32_t);
    // memcpy(buffer, &msg_raw->stamp.sec, sizeof(long int));
    // buffer = buffer + sizeof(long int);
    // offset = offset + sizeof(long int);
    // memcpy(buffer, &msg_raw->stamp.nsec, sizeof(long int));
    // buffer = buffer + sizeof(long int);
    // offset = offset + sizeof(long int);
    // //frame_id
    // // uint8_t idLen = msg_raw->frame_id.length();
    // uint8_t idLen = FRAME_IDS_MAX;
    // memcpy(buffer, &idLen, sizeof(uint8_t));//frame_id长度
    // buffer = buffer + sizeof(uint8_t);
    // offset = offset + sizeof(uint8_t);
    // for(uint8_t i = 0; i < idLen; i++)
    // {
    //     uint8_t strEle = (uint8_t)msg_raw->frame_id.at(i);//不包括"/0"
    //     memcpy(buffer, &strEle, sizeof(uint8_t));
    //     buffer = buffer + sizeof(uint8_t);
    //     offset = offset + sizeof(uint8_t);
    // }
    
    // uint8_t restStrLen= strBufferLen - idLen - 2;
    // if(restStrLen < 0)
    // {
    //     return -1;
    // }
    // memcpy(buffer, &restStrLen, sizeof(uint8_t));
    // buffer = buffer + sizeof(uint8_t);
    // offset = offset + sizeof(uint8_t);

    // memset(buffer, 0, restStrLen);
    // buffer = buffer + restStrLen * sizeof(uint8_t);
    // offset = offset + restStrLen * sizeof(uint8_t);
    
    // *length = offset;

    // return 0;
}
int sensorHeaderDeSerializer(uint8_t *buffer, uint32_t *length, TSensorMsgHeader *msg_raw)
{

    if (buffer == NULL || length == NULL || msg_raw == NULL)
    {
        return -1;
    }

    memcpy(msg_raw, buffer, sizeof(TSensorMsgHeader));

    return 0;
    // uint8_t idLen = 0;
    // std::string frame_id;
    // uint32_t offset = 0;
    // uint8_t restLen = 0;

    // memcpy(&msg_raw->seq, buffer, sizeof(uint32_t));
    // buffer = buffer + sizeof(uint32_t);
    // offset = offset + sizeof(uint32_t);
    // memcpy(&msg_raw->stamp.sec, buffer, sizeof(long int));
    // buffer = buffer + sizeof(long int);
    // offset = offset + sizeof(long int);
    // memcpy(&msg_raw->stamp.nsec, buffer, sizeof(long int));
    // buffer = buffer + sizeof(long int);
    // offset = offset + sizeof(long int);
    // //
    // memcpy(&idLen, buffer, sizeof(uint8_t));
    // buffer = buffer + sizeof(uint8_t);
    // offset = offset + sizeof(uint8_t);
    // for(uint8_t i = 0; i < idLen; i++)
    // {
    //     msg_raw->frame_id.push_back((char )(*buffer));
    //     buffer = buffer + sizeof(uint8_t);
    //     offset = offset + sizeof(uint8_t);
    // }
    // restLen = *buffer;
    // buffer = buffer + (restLen + 1) * sizeof(uint8_t);
    // offset = offset + (restLen + 1) * sizeof(uint8_t);

    // *length = offset;

    //return 0;
}
int laserMsgSerializer(uint8_t *buffer, uint32_t *length, TAgvLaserScanMsg *msg_raw)
{
    // uint32_t sensorHeadLen = 0;
    // uint32_t offset = 0;
    // //激光头序列化
    // sensorHeaderSerializer(buffer, &sensorHeadLen, &msg_raw->header);
    // buffer = buffer + sensorHeadLen * sizeof(uint8_t);//头序列化后buffer又指向buffer[0],需要进行偏移
    // //body序列化
    // memcpy(buffer, &msg_raw->angle_min, sizeof(float));
    // buffer = buffer + sizeof(float);
    // offset = offset + sizeof(float);
    // memcpy(buffer, &msg_raw->angle_max, sizeof(float));
    // buffer = buffer + sizeof(float);
    // offset = offset + sizeof(float);
    // memcpy(buffer, &msg_raw->angle_increment, sizeof(float));
    // buffer = buffer + sizeof(float);
    // offset = offset + sizeof(float);
    // memcpy(buffer, &msg_raw->time_increment, sizeof(float));
    // buffer = buffer + sizeof(float);
    // offset = offset + sizeof(float);
    // memcpy(buffer, &msg_raw->scan_time, sizeof(float));
    // buffer = buffer + sizeof(float);
    // offset = offset + sizeof(float);
    // memcpy(buffer, &msg_raw->range_min, sizeof(float));
    // buffer = buffer + sizeof(float);
    // offset = offset + sizeof(float);
    // memcpy(buffer, &msg_raw->range_max, sizeof(float));
    // buffer = buffer + sizeof(float);
    // offset = offset + sizeof(float);

    // uint32_t rangeSize = msg_raw->ranges.size();
    // memcpy(buffer, &rangeSize, sizeof(uint32_t));
    // buffer = buffer +  sizeof(uint32_t);
    // offset = offset +  sizeof(uint32_t);
    // for(uint32_t i = 0; i < rangeSize; i++)
    // {   
    //     float rangeData = msg_raw->ranges.at(i);
    //     memcpy(buffer, &rangeData, sizeof(float));
    //     buffer = buffer +  sizeof(float);
    //     offset = offset +  sizeof(float);
    // }
    
    // uint32_t intenSize = msg_raw->intensities.size();
    // memcpy(buffer, &intenSize, sizeof(uint32_t));
    // buffer = buffer +  sizeof(uint32_t);
    // offset = offset +  sizeof(uint32_t);

    // for(uint32_t j = 0; j < intenSize; j++)
    // {
    //     float intenData = msg_raw->intensities.at(j);
    //     memcpy(buffer, &intenData, sizeof(float));
    //     buffer = buffer +  sizeof(float);
    //     offset = offset +  sizeof(float);
    // }

    // *length = sensorHeadLen + offset;

    return 0;
}
int laserMsgDeserializer(uint8_t *buffer, uint32_t *length, TAgvLaserScanMsg *msg_raw)
{
    // uint32_t sensorHeadLen = 0;
    // uint32_t offset = 0;
    // float rangeData;
    // float intenData;
    // //激光头反序列化
    // sensorHeaderDeSerializer(buffer, &sensorHeadLen, &msg_raw->header);
    // buffer = buffer + sensorHeadLen * sizeof(uint8_t);
    // //body反序列化
    // memcpy(&msg_raw->angle_min, buffer, sizeof(float));
    // buffer = buffer + sizeof(float);
    // offset = offset + sizeof(float);
    // memcpy(&msg_raw->angle_max, buffer, sizeof(float));
    // buffer = buffer + sizeof(float);
    // offset = offset + sizeof(float);
    // memcpy(&msg_raw->angle_increment, buffer, sizeof(float));
    // buffer = buffer + sizeof(float);
    // offset = offset + sizeof(float);
    // memcpy(&msg_raw->time_increment, buffer, sizeof(float));
    // buffer = buffer + sizeof(float);
    // offset = offset + sizeof(float);
    // memcpy(&msg_raw->scan_time, buffer, sizeof(float));
    // buffer = buffer + sizeof(float);
    // offset = offset + sizeof(float);
    // memcpy(&msg_raw->range_min, buffer, sizeof(float));
    // buffer = buffer + sizeof(float);
    // offset = offset + sizeof(float);
    // memcpy(&msg_raw->range_max, buffer, sizeof(float));
    // buffer = buffer + sizeof(float);
    // offset = offset + sizeof(float);

    // uint32_t rangeSize;
    // memcpy(&rangeSize, buffer, sizeof(uint32_t));
    // buffer = buffer +  sizeof(uint32_t);
    // offset = offset +  sizeof(uint32_t);
    // for(uint32_t i = 0; i < rangeSize; i++)
    // {   
    //     memcpy(&rangeData, buffer, sizeof(float));
    //     msg_raw->ranges.push_back(rangeData);
    //     buffer = buffer +  sizeof(float);
    //     offset = offset +  sizeof(float);
    // }

    // uint32_t intenSize;
    // memcpy(&intenSize, buffer, sizeof(uint32_t));
    // buffer = buffer +  sizeof(uint32_t);
    // offset = offset +  sizeof(uint32_t);

    // for(uint32_t j = 0; j < intenSize; j++)
    // {
    //     memcpy(&intenData, buffer, sizeof(float));
    //     msg_raw->intensities.push_back(intenData);
    //     buffer = buffer +  sizeof(float);
    //     offset = offset +  sizeof(float);
    // }

    // *length = sensorHeadLen + offset;

    return 0;
}
int imuMsgSerializer(uint8_t *buffer, uint32_t *length, TAgvImuMsg *msg_raw)
{
    uint32_t sensorHeadLen = 0;
    uint32_t offset = 0;
    //激光头序列化
    sensorHeaderSerializer(buffer, &sensorHeadLen, &msg_raw->header);
    buffer = buffer + sensorHeadLen * sizeof(uint8_t);
    //body序列化
    //orientation
    memcpy(buffer, &msg_raw->orientation.x, sizeof(double));
    buffer = buffer + sizeof(double);
    offset = offset + sizeof(double);
    memcpy(buffer, &msg_raw->orientation.y, sizeof(double));
    buffer = buffer + sizeof(double);
    offset = offset + sizeof(double);
    memcpy(buffer, &msg_raw->orientation.z, sizeof(double));
    buffer = buffer + sizeof(double);
    offset = offset + sizeof(double);
    memcpy(buffer, &msg_raw->orientation.w, sizeof(double));
    buffer = buffer + sizeof(double);
    offset = offset + sizeof(double);
    //angular_velocity
    memcpy(buffer, &msg_raw->angular_velocity.x, sizeof(double));
    buffer = buffer + sizeof(double);
    offset = offset + sizeof(double);
    memcpy(buffer, &msg_raw->angular_velocity.y, sizeof(double));
    buffer = buffer + sizeof(double);
    offset = offset + sizeof(double);
    memcpy(buffer, &msg_raw->angular_velocity.z, sizeof(double));
    buffer = buffer + sizeof(double);
    offset = offset + sizeof(double);
    //linear_acceleration
    memcpy(buffer, &msg_raw->linear_acceleration.x, sizeof(double));
    buffer = buffer + sizeof(double);
    offset = offset + sizeof(double);
    memcpy(buffer, &msg_raw->linear_acceleration.y, sizeof(double));
    buffer = buffer + sizeof(double);
    offset = offset + sizeof(double);
    memcpy(buffer, &msg_raw->linear_acceleration.z, sizeof(double));
    buffer = buffer + sizeof(double);
    offset = offset + sizeof(double);

    *length = sensorHeadLen + offset;

    return 0;
}

int imuMsgDeserializer(uint8_t *buffer, uint32_t *length, TAgvImuMsg *msg_raw)
{
    uint32_t sensorHeadLen = 0;
    uint32_t offset = 0;
    //imu头反序列化
    sensorHeaderDeSerializer(buffer, &sensorHeadLen, &msg_raw->header);
    buffer = buffer + sensorHeadLen * sizeof(uint8_t);
    //body反序列化
    memcpy(&msg_raw->orientation.x, buffer, sizeof(double));//orientation.x
    buffer = buffer + sizeof(double);
    offset = offset + sizeof(double);
    memcpy(&msg_raw->orientation.y, buffer, sizeof(double));//orientation.y
    buffer = buffer + sizeof(double);
    offset = offset + sizeof(double);
    memcpy(&msg_raw->orientation.z, buffer, sizeof(double));//orientation.z
    buffer = buffer + sizeof(double);
    offset = offset + sizeof(double);
    memcpy(&msg_raw->orientation.w, buffer, sizeof(double));//orientation.w
    buffer = buffer + sizeof(double);
    offset = offset + sizeof(double);
    memcpy(&msg_raw->angular_velocity.x, buffer, sizeof(double));//angular_velocity.x
    buffer = buffer + sizeof(double);
    offset = offset + sizeof(double);
    memcpy(&msg_raw->angular_velocity.y, buffer, sizeof(double));//angular_velocity.y
    buffer = buffer + sizeof(double);
    offset = offset + sizeof(double);
    memcpy(&msg_raw->angular_velocity.z, buffer, sizeof(double));//angular_velocity.z
    buffer = buffer + sizeof(double);
    offset = offset + sizeof(double);

    memcpy(&msg_raw->linear_acceleration.x, buffer, sizeof(double));//linear_acceleration.x
    buffer = buffer + sizeof(double);
    offset = offset + sizeof(double);
    memcpy(&msg_raw->linear_acceleration.y, buffer, sizeof(double));//linear_acceleration.y
    buffer = buffer + sizeof(double);
    offset = offset + sizeof(double);
    memcpy(&msg_raw->linear_acceleration.z, buffer, sizeof(double));//linear_acceleration.z
    buffer = buffer + sizeof(double);
    offset = offset + sizeof(double);

    *length = sensorHeadLen + offset;

    return 0;
}

int odomMsgSerializer(uint8_t *buffer, uint32_t *length, TAgvOdomMsg *msg_raw)
{   
    // uint32_t sensorHeadLen = 0;
    // uint32_t offset = 0;
    // uint8_t strBufferLen = 20;
    // //激光头序列化
    // sensorHeaderSerializer(buffer, &sensorHeadLen, &msg_raw->header);
    // buffer = buffer + sensorHeadLen * sizeof(uint8_t);
    // //body序列化
    // //child_frame_id
    // uint8_t idLen = msg_raw->child_frame_id.length();
    // memcpy(buffer, &idLen, sizeof(uint8_t));//child_frame_id长度
    // buffer = buffer + sizeof(uint8_t);
    // offset = offset + sizeof(uint8_t);
    // for(uint8_t i = 0; i < idLen; i++)
    // {
    //     uint8_t strEle = (uint8_t)msg_raw->child_frame_id.at(i);//不包括"/0"
    //     memcpy(buffer, &strEle, sizeof(uint8_t));
    //     buffer = buffer + sizeof(uint8_t);
    //     offset = offset + sizeof(uint8_t);
    // }
    
    // uint8_t restStrLen= strBufferLen - idLen - 2;
    // if(restStrLen < 0)
    // {
    //     return -1;
    // }
    // memcpy(buffer, &restStrLen, sizeof(uint8_t));
    // buffer = buffer + sizeof(uint8_t);
    // offset = offset + sizeof(uint8_t);

    // memset(buffer, 0, restStrLen);
    // buffer = buffer + restStrLen * sizeof(uint8_t);
    // offset = offset + restStrLen * sizeof(uint8_t);
    // //TAgvPose3D
    // memcpy(buffer, &msg_raw->pose.position.x, sizeof(double));
    // buffer = buffer + sizeof(double);
    // offset = offset + sizeof(double);
    // memcpy(buffer, &msg_raw->pose.position.y, sizeof(double));
    // buffer = buffer + sizeof(double);
    // offset = offset + sizeof(double);
    // memcpy(buffer, &msg_raw->pose.position.z, sizeof(double));
    // buffer = buffer + sizeof(double);
    // offset = offset + sizeof(double);
    
    // memcpy(buffer, &msg_raw->pose.orientation.x, sizeof(double));
    // buffer = buffer + sizeof(double);
    // offset = offset + sizeof(double);
    // memcpy(buffer, &msg_raw->pose.orientation.y, sizeof(double));
    // buffer = buffer + sizeof(double);
    // offset = offset + sizeof(double);
    // memcpy(buffer, &msg_raw->pose.orientation.z, sizeof(double));
    // buffer = buffer + sizeof(double);
    // offset = offset + sizeof(double);
    // memcpy(buffer, &msg_raw->pose.orientation.w, sizeof(double));
    // buffer = buffer + sizeof(double);
    // offset = offset + sizeof(double);
    // //TAgvTwist3D
    // memcpy(buffer, &msg_raw->twist.linear.x, sizeof(double));
    // buffer = buffer + sizeof(double);
    // offset = offset + sizeof(double);
    // memcpy(buffer, &msg_raw->twist.linear.y, sizeof(double));
    // buffer = buffer + sizeof(double);
    // offset = offset + sizeof(double);
    // memcpy(buffer, &msg_raw->twist.linear.z, sizeof(double));
    // buffer = buffer + sizeof(double);
    // offset = offset + sizeof(double);

    // memcpy(buffer, &msg_raw->twist.angular.x, sizeof(double));
    // buffer = buffer + sizeof(double);
    // offset = offset + sizeof(double);
    // memcpy(buffer, &msg_raw->twist.angular.y, sizeof(double));
    // buffer = buffer + sizeof(double);
    // offset = offset + sizeof(double);
    // memcpy(buffer, &msg_raw->twist.angular.z, sizeof(double));
    // buffer = buffer + sizeof(double);
    // offset = offset + sizeof(double);

    // *length = offset;

    return 0;
}

int odomMsgDeserializer(uint8_t *buffer, uint32_t *length, TAgvOdomMsg *msg_raw)
{
    // uint32_t sensorHeadLen = 0;
    // uint32_t offset = 0;
    // uint8_t idLen = 0;
    // uint8_t restLen = 0;
    // //odom头反序列化
    // sensorHeaderDeSerializer(buffer, &sensorHeadLen, &msg_raw->header);
    // buffer = buffer + sensorHeadLen * sizeof(uint8_t);
    // //body反序列化
    // //child_frame_id
    // memcpy(&idLen, buffer, sizeof(uint8_t));
    // buffer = buffer + sizeof(uint8_t);
    // offset = offset + sizeof(uint8_t);
    // for(uint8_t i = 0; i < idLen; i++)
    // {
    //     msg_raw->child_frame_id.push_back((char )(*buffer));
    //     buffer = buffer + sizeof(uint8_t);
    //     offset = offset + sizeof(uint8_t);
    // }
    // restLen = *buffer;
    // buffer = buffer + (restLen + 1) * sizeof(uint8_t);
    // offset = offset + (restLen + 1) * sizeof(uint8_t);

    // //TAgvPose3D
    // memcpy(&msg_raw->pose.position.x, buffer, sizeof(double));
    // buffer = buffer + sizeof(double);
    // offset = offset + sizeof(double);
    // memcpy(&msg_raw->pose.position.y, buffer, sizeof(double));
    // buffer = buffer + sizeof(double);
    // offset = offset + sizeof(double);
    // memcpy(&msg_raw->pose.position.z, buffer, sizeof(double));
    // buffer = buffer + sizeof(double);
    // offset = offset + sizeof(double);
    
    // memcpy(&msg_raw->pose.orientation.x, buffer, sizeof(double));
    // buffer = buffer + sizeof(double);
    // offset = offset + sizeof(double);
    // memcpy(&msg_raw->pose.orientation.y, buffer, sizeof(double));
    // buffer = buffer + sizeof(double);
    // offset = offset + sizeof(double);
    // memcpy(&msg_raw->pose.orientation.z, buffer, sizeof(double));
    // buffer = buffer + sizeof(double);
    // offset = offset + sizeof(double);
    // memcpy(&msg_raw->pose.orientation.w, buffer, sizeof(double));
    // buffer = buffer + sizeof(double);
    // offset = offset + sizeof(double);
    // //TAgvTwist3D
    // memcpy(&msg_raw->twist.linear.x, buffer, sizeof(double));
    // buffer = buffer + sizeof(double);
    // offset = offset + sizeof(double);
    // memcpy(&msg_raw->twist.linear.y, buffer, sizeof(double));
    // buffer = buffer + sizeof(double);
    // offset = offset + sizeof(double);
    // memcpy(&msg_raw->twist.linear.z, buffer, sizeof(double));
    // buffer = buffer + sizeof(double);
    // offset = offset + sizeof(double);

    // memcpy(&msg_raw->twist.angular.x, buffer, sizeof(double));
    // buffer = buffer + sizeof(double);
    // offset = offset + sizeof(double);
    // memcpy(&msg_raw->twist.angular.y, buffer, sizeof(double));
    // buffer = buffer + sizeof(double);
    // offset = offset + sizeof(double);
    // memcpy(&msg_raw->twist.angular.z, buffer, sizeof(double));
    // buffer = buffer + sizeof(double);
    // offset = offset + sizeof(double);

    // *length = offset;

    return 0;
}

int OccupancyGridMsgSerializer(uint8_t *buffer, uint32_t *length, TOccupancyGrid *msg_raw)
{
    uint32_t sensorHeadLen = 0;
    uint32_t offset = 0;
    //栅格地图头序列化
    sensorHeaderSerializer(buffer, &sensorHeadLen, &msg_raw->header);
    buffer = buffer + sensorHeadLen * sizeof(uint8_t);

    memcpy(buffer, &msg_raw->map_load_time.sec, sizeof(long int));
    buffer = buffer + sizeof(long int);
    offset = offset + sizeof(long int);
    memcpy(buffer, &msg_raw->map_load_time.nsec, sizeof(long int));
    buffer = buffer + sizeof(long int);
    offset = offset + sizeof(long int);

    memcpy(buffer, &msg_raw->resolution, sizeof(double));
    buffer = buffer + sizeof(double);
    offset = offset + sizeof(double);

    memcpy(buffer, &msg_raw->width, sizeof(uint32_t));
    buffer = buffer + sizeof(uint32_t);
    offset = offset + sizeof(uint32_t);

    memcpy(buffer, &msg_raw->height, sizeof(uint32_t));
    buffer = buffer + sizeof(uint32_t);
    offset = offset + sizeof(uint32_t);

    memcpy(buffer, &msg_raw->origin, sizeof(TAgvPose3D));
    buffer = buffer + sizeof(TAgvPose3D);
    offset = offset + sizeof(TAgvPose3D);

    for(uint32_t i = 0; i < (msg_raw->width * msg_raw->height); i++)
    {   
        int8_t occu_data = msg_raw->data.at(i);
        memcpy(buffer, &occu_data, sizeof(int8_t));
        buffer = buffer +  sizeof(int8_t);
        offset = offset +  sizeof(int8_t);
    }

    *length = sensorHeadLen + offset;

    return 0;
}
int OccupancyGridMsgDeserializer(uint8_t *buffer, uint32_t *length, TOccupancyGrid *msg_raw)
{
    uint32_t sensorHeadLen = 0;
    uint32_t offset = 0;
    int8_t occu_data = 0;
    //激光头反序列化
    sensorHeaderDeSerializer(buffer, &sensorHeadLen, &msg_raw->header);
    buffer = buffer + sensorHeadLen * sizeof(uint8_t);
    //body反序列化
    memcpy(&msg_raw->map_load_time.sec, buffer, sizeof(long int));
    buffer = buffer + sizeof(long int);
    offset = offset + sizeof(long int);
    memcpy(&msg_raw->map_load_time.nsec, buffer, sizeof(long int));
    buffer = buffer + sizeof(long int);
    offset = offset + sizeof(long int);

    memcpy(&msg_raw->resolution, buffer, sizeof(double));
    buffer = buffer + sizeof(double);
    offset = offset + sizeof(double);

    memcpy(&msg_raw->width, buffer, sizeof(uint32_t));
    buffer = buffer + sizeof(uint32_t);
    offset = offset + sizeof(uint32_t);

    memcpy(&msg_raw->height, buffer, sizeof(uint32_t));
    buffer = buffer + sizeof(uint32_t);
    offset = offset + sizeof(uint32_t);

    memcpy(&msg_raw->origin, buffer, sizeof(TAgvPose3D));
    buffer = buffer + sizeof(TAgvPose3D);
    offset = offset + sizeof(TAgvPose3D);

    for(uint32_t i = 0; i < (msg_raw->width * msg_raw->height); i++)
    {   
        memcpy(&occu_data, buffer, sizeof(int8_t));
        msg_raw->data.push_back(occu_data);
        buffer = buffer +  sizeof(int8_t);
        offset = offset +  sizeof(int8_t);
    }

    *length = sensorHeadLen + offset;
    
    return 0;
}