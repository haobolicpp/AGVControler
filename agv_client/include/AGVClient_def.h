/******************************************************************************
*
* 文件名称：AGVClient_def.h
*
******************************************************************************/
#pragma once
#include <string>
#include <map>
#include <unordered_map>
#include <vector>
#include <list>
#include "GeometryAlgorithm.h"

/////////////////////
//站点类型
enum class EStationType{
    StationType_General, //普通站点
    StationType_Charge   //充电站点
};

//路线方向
enum class EPathDirect{
    PathDirect_StartToEnd,
    PathDirect_EndToStart,
    PathDirect_Bothway      //双向
};

//路线类型
enum class EPathType{
    PathType_Line, //直线
    PathType_Circle, //圆
    PathType_Bezier  //三阶贝塞尔曲线
};

//站点
typedef struct TStation{
    int iStationID; //站点编号
    EStationType eType; //类型
    TPointd tPt; //当前坐标，scene像素坐标，注意(0,0)位置
    double dAngle; //朝向角，°范围(0~180 0~-180)
    std::string strComment; //注释信息
}TStation;

//圆弧扩展数据
typedef struct TCircleArcExtraData{
    TPointd tPtCenter; //圆心 像素坐标
    bool bIsGood; //是否是优弧
}TCircleArcExtraData;

//三阶贝塞尔曲线扩展数据，通过公式来差值坐标点
typedef struct TBazierExtraData{
    TPointd tPtNearStart; //靠近起点的点，像素坐标
    TPointd tPtNearEnd; //靠近终点的点
}TBazierExtraData;

//路线
typedef struct TPath{
    int iPathID; //路线编号
    EPathType eType; //路线类型
    int iStartStation; //起始站点
    int iEndStation;  //终止站点
    TPointd tStartToEndPt; //连接起始站点的线段另一个端点坐标
    TPointd tEndToEndPt;  //连接终止站点的线段另一个端点坐标
    union {
        TCircleArcExtraData tArcData; //圆弧数据
        TBazierExtraData tBazierData; //贝塞尔曲线数据
    }udata;
    EPathDirect eDirect; //路线方向
}TPath;

//地图信息
typedef struct TMapInfo{
    //地图基础信息
    std::string strMapName; //地图名称
    std::string strVersion; //地图版本

    //网格地图信息
    //unsigned char *pData; //收到数据后墙砖为两个vector，地图数据：0空白，100占用，255未知
    double dResolution; //分辨率 m
    int iHight; //高度(像素)
    int iWidth;  //宽度(像素)
    double dOriginXOffset; //地图左下角x偏移，由此可计算出地图(0,0)点位置，单位m
    double dOriginYOffset; //地图左下角y偏移
    double dTheta; //地图绕(0,0)点逆时针旋转角度（暂时作用是UI调整地图后存储的角度，0度则地图不进行旋转变换）
    std::vector<TPoint> vecUnknown;//地图绘制加速映射表——未知点集(坐标系是以地图左上角为0，0点；不是场景的0,0点)
    std::vector<TPoint> vecOccupy;//地图绘制加速映射表——占用点集(障碍点)

    //拓扑路线信息key:pathid
    std::map<int, TPath> mapPath;

    //地图的站点信息
    std::map<int, TStation> mapStation;

}TMapInfo;


////////////////AGV信息
//连接状态
enum class EConnectStatus{
    Connected_Invalid, //初始状态
    Connected_False,   //连接失败
    Connected_Sucess   //连接成功
};

//AGV控制器的运行状态
enum class EAGVRunMode{
    RunMode_Free,     //空闲
    RunMode_Nav,      //导航
    RunMode_Mapping   //建图
};

struct TMapInfo;

//AGV信息
typedef struct TAGVInfo
{
    //主页列表
    std::string strAGVID;  //AGV编号
    std::string strAGVIP;   //AGV ip地址
    EConnectStatus eStatus; //AGV连接状态
    int iElectricity;  //电量 %
    double dConfidence;  //置信度 0~1
    //int iPing;  //网络延迟 ms
    bool bIsSync;  //地图及数据是否是同步的
    std::string strMapName; //本地存储的地图名称
    std::string strComment; //备注

    //其他信息
    double dLineSpeed; //线速度 m/s
    double dAngularSpeed; //角速度 rad/s
    TPointd tPt; //当前坐标 世界坐标系
    double dAngle;//车头朝向角(° -180~180)
    std::vector<TPointd> vecDWALocalPath;// dwa实时局部路径

    //地图信息
    std::string strMapJson;//地图json数据字符串，下发文件时使用
    //TMapInfo tMapInfo; //地图数据，初始化时解析json得到，扫描地图后也存储到这里
    std::string strMapMD5; //地图文件的MD5值
    //bool bIsJsonMapInfoSync; //json地图、TMapInfo是否同步过，第一次打开某个AGV页面时false

    //AGV控制器信息
    int iRAM; //内存占用百分比
    int iCPU; //CPU占用
    std::string strVersion; //控制器程序版本号
    EAGVRunMode eAGVRunMode; //控制器的运行状态
    std::string strControllerMapName;   //AGV控制器正在使用的地图
    std::string strControllerMapMD5;    //AGV控制器正在使用的地图的MD5

    //UI临时信息
    bool bChecked; //是否被选中

}TAGVInfo;

//地图文件中的地图数据定义
#define MAP_EMPTY '0'
#define MAP_UNKNOWN '1' //未知
#define MAP_OCCYPY '2'  //占用




