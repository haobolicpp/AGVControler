/*
 * @Author: yang.cheng yang.cheng@bgirobot.com
 * @Date: 2023-04-10 08:32:38
 * @LastEditors: yang.cheng yang.cheng@bgirobot.com
 * @LastEditTime: 2023-09-12 16:02:32
 * @FilePath: /agv_controller/include/agv_common/agv_type.h
 * @Description: 基础类型定义
 */
#ifndef AGV_TYPE_H
#define AGV_TYPE_H

#include <stdio.h>
#include <stdint.h>
#include <string>
#include <vector>
#include <map>
#include <time.h>

#ifndef PI
#define PI 3.1415926
#endif
//坐标
typedef struct TPointd{
    double dX; //X坐标
    double dY; //Y坐标
}TPointd;
typedef struct TPoint{
    int iX; //X坐标
    int iY; //Y坐标
}TPoint;

//AGV 基本信息
class TBaseInfo
{
    public:
    std::string strAGVID;  //AGV编号
    //std::string strAGVIP;   //AGV ip地址
    //EConnectStatus eStatus; //AGV连接状态
    int iElectricity;  //电量 %
    double dConfidence;  //置信度 0~1
    int iPing;  //网络延迟 ms
    bool bIsSync;  //地图及数据是否是同步的
    //std::string strMapName; //存储的地图名称
    //std::string strComment; //备注

    //AGV控制器信息
    int iRAM; //内存占用百分比
    int iCPU; //CPU占用
    //std::string strVersion; //控制器程序版本号

    //其他信息
    double dLineSpeed; //线速度 m/s
    double dAngularSpeed; //角速度 rad/s
    TPointd tPt; //当前坐标
    double dAngle;//车头朝向角(° -PI~PI)
    double dLastAngle; //上一次的角度（默认-90度）
    //std::vector<TPointd> vecDWALocalPath;// dwa实时局部路径
};//TBaseInfo;

//必要的声明
class AgvCtrl;


//兼容老版本的TAgvPose->TAgvPose2D
typedef struct
{
    double fx;
    double fy;
    double ftheta;
}TAgvPose;

//通用3元浮点类型组
typedef struct AgvVector3D
{
    double x;
    double y;
    double z;
}TAgvVector3D;

//AGV 3D位置
typedef struct AgvPosion3D
{
    double x;
    double y;
    double z;
}TAgvPosition3D;


//AGV 3D位置
typedef struct AgvPosion3f
{
    float x;
    float y;
    float z;
}TAgvPosition3f;

//AGV 方位四元数
typedef struct AgvOrientation
{
    double x;
    double y;
    double z;
    double w;
}TAgvOrientation;

//AGV 3D位姿结构
typedef struct AgvPose3D
{
    TAgvPosition3D position;
    TAgvOrientation orientation;
}TAgvPose3D;

//AGV 3D Twist
typedef struct AgvTwist3D
{
    TAgvVector3D linear; //线速度
    TAgvVector3D angular; //角速度（r p y排列）
}TAgvTwist3D;

//AGV 2D位姿描述
typedef struct AgvPose2D
{
    double x;
    double y;
    double phi;
}TAgvPose2D;

//AGV 2D Twist
typedef struct AgvTwist2D
{
    double v; //线速度
    double w; //角速度
}TAgvTwist2D;

//AGV 2D 规划参考点类型 包含点的速度
typedef struct AgvMotionNode2D
{
    TAgvPose2D tPose2D;
    TAgvTwist2D tTwist2D;
}TAgvMotionNode;
//传感器数据的时间戳结构
typedef struct AgvTimeStamp
{
    long int sec;
    long int nsec;
}TAgvTimeStamp;

//AGV时间戳位姿
typedef struct AgvPose3DStamped
{
    TAgvPose3D pose;
}TAgvPose3DStamped;


//v1.02 By yang.cheng
//使用C语言风格 结构体严格按照C要求 不再使用STD容器
//便于序列化与消息构建
#define FRAME_IDS_MAX 64
//传感器数据的消息头定义
typedef struct SensorMsgHeader
{
    uint32_t seq;
    TAgvTimeStamp stamp;
    // std::string frame_id;
    char frame_id[FRAME_IDS_MAX];

}TSensorMsgHeader;

//里程计传感器数据的格式定义
typedef struct AgvOdomMsg
{
    TSensorMsgHeader header;
    char child_frame_id[FRAME_IDS_MAX];
    TAgvPose3D pose;
    TAgvTwist3D twist;
}TAgvOdomMsg;

//IMU传感器数据的格式定义
typedef struct AgvImuMsg
{
    TSensorMsgHeader header;
    TAgvOrientation orientation;
    TAgvVector3D angular_velocity;
    TAgvVector3D linear_acceleration;
}TAgvImuMsg;


//激光雷达传感器数据的格式定义
#define LASER_RANGES_MAX 2048
typedef struct AgvLaserScanMsg
{
    TSensorMsgHeader header;
    float angle_min;
    float angle_max;
    float angle_increment;
    float time_increment;
    float scan_time;
    float range_min;
    float range_max;
    // std::vector<float> ranges;
    // std::vector<float> intensities;
    int   ranges_size;
    float ranges[LASER_RANGES_MAX];
    int   intensities_size;
    float intensities[LASER_RANGES_MAX];
}TAgvLaserScanMsg;

//点云数据的格式定义
typedef struct AgvPointCloud2
{   
    TSensorMsgHeader header;
    std::vector<TAgvPosition3D> points;
}TAgvPointCloud2;

//子图列表
typedef struct SubmapEntry
{
    int8_t trajectory_id;
    int submap_index;
    int submap_version;
    TAgvPose3D pose;
    bool is_frozen;
}TSubmapEntry;

typedef struct SubmapList{
    TSensorMsgHeader header;
    std::vector<TSubmapEntry> submap;
} TSubmapList;

typedef struct SubmapTexture{
    std::vector<int8_t> cells;
    int32_t width;
    int32_t height;
    float resolution;
    TAgvPose3D slice_pose;
}TSubmapTexture;

typedef struct SubmapQuery{
    int32_t submap_version;
    std::vector<TSubmapTexture> textures;
}TSubmapQuery;

//栅格地图数据结构
typedef struct OccupancyGrid{
    TSensorMsgHeader header;
    TAgvTimeStamp map_load_time;
    double resolution;
    uint32_t width;
    uint32_t height;
    TAgvPose3D origin;
    std::vector<int8_t> data;
}TOccupancyGrid;


//最大允许的序列化长度
#define SERIALIZE_LENGTH_MAX 2*1024*1024
#define OCCUPANCY_GRID_BASE_LEN (sizeof(TSensorMsgHeader) + \
                                sizeof(TAgvTimeStamp) + \
                                sizeof(double) + \
                                2*sizeof(uint32_t) + \
                                sizeof(TAgvPose3D))

#define POINT_CLOUD_BASE_LEN sizeof(TSensorMsgHeader)

/**
 * @name: SerializeOccupancyGrid
 * @des:  序列化地图数据用于传输
 * @param {TOccupancyGrid} &tOccupancyGrid
 * @param {uint8_t} *pSerializeOut
 * @param {int} &s32Len
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int SerializeOccupancyGrid(TOccupancyGrid &tOccupancyGrid, uint8_t *pSerializeOut, int &s32LenOut);
/**
 * @name: DeSerilizeOccupancyGrid
 * @des:  反序列化地图数据
 * @param {uint8_t} *pSerializeIn
 * @param {int} &s32LenIn
 * @param {TOccupancyGrid} &tOccupancyGrid
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int DeSerilizeOccupancyGrid(uint8_t *pSerializeIn, int &s32LenIn, TOccupancyGrid &tOccupancyGrid);
/**
 * @name: 
 * @des: 
 * @param {TAgvPointCloud2} &tPointCloud2
 * @param {uint8_t} *pSerializeOut
 * @param {int} &s32LenOut
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int SerializePointCloud2(TAgvPointCloud2 &tPointCloud2, uint8_t *pSerializeOut, int &s32LenOut);
/**
 * @name: 
 * @des: 
 * @param {uint8_t} *pSerializeIn
 * @param {int} &s32LenIn
 * @param {TAgvPointCloud2} &tPointCloud2
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int DeSerializePointCloud2(uint8_t *pSerializeIn, int &s32LenIn, TAgvPointCloud2 &tPointCloud2);

#endif // AGV_TYPE_H