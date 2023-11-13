#ifndef AGV_ROS_CTRL_H
#define AGV_ROS_CTRL_H

#include <map>
#include <list>
#include <pthread.h>
#include <ros/ros.h>
#include <nav_msgs/GetMap.h>
#include <agv_comm/sys_queue.h>
#include <agv_comm/agv_type.h>

#include "cartographer_ros_msgs/ChangeMode.h"
#include "cartographer_ros_msgs/StopPubMap.h"
#include "cartographer_ros_msgs/StatusCode.h"
#include "cartographer_ros_msgs/StatusResponse.h"

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>

#include <move_base_msgs/MoveBase_GlobalPathAction.h>

//MoveBase Client Define
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>  MoveBaseClient;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBase_GlobalPathAction>  MoveBaseClient_GlobalPath;

/////////////////////////////
//AGV Ctrl 结构的状态
enum class E_AGV_CTRL_STATE
{
    E_AGV_CTRL_INIT = 0,
    E_AGV_CTRL_NAVI = 1,
    E_AGV_CTRL_SLAM = 2,
    E_AGV_CTRL_ERROR = 3,
    E_AGV_CTRL_MAX

};


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
    TPointd tPt; //当前坐标，注意(0,0)位置在图像中心
    double dAngle; //朝向角，°范围(0~180 0~-180)
    std::string strComment; //注释信息
}TStation;

//圆弧扩展数据
typedef struct TCircleArcExtraData{
    TPointd tPtCenter; //圆心 像素坐标，注意(0,0)位置在图像中心
    bool bIsGood; //是否是优弧
}TCircleArcExtraData;

//三阶贝塞尔曲线扩展数据，通过公式来差值坐标点
typedef struct TBazierExtraData{
    TPointd tPtNearStart; //靠近起点的点，像素坐标，注意(0,0)位置在图像中心
    TPointd tPtNearEnd; //靠近终点的点
}TBazierExtraData;

//路线
typedef struct TPath{
    int iPathID; //路线编号
    EPathType eType; //路线类型
    int iStartStation; //起始站点
    int iEndStation;  //终止站点
    union {
        TCircleArcExtraData tArcData; //圆弧数据
        TBazierExtraData tBazierData; //贝塞尔曲线数据
    }udata;
    EPathDirect eDirect; //路线方向
}TPath;

//地图信息
typedef struct MapInfo{
    //地图基础信息
    std::string strMapName; //地图名称
    std::string strVersion; //地图版本

    //网格地图信息
    double dResolution; //分辨率 m
    int iHeight; //高度(像素)
    int iWidth;  //宽度(像素)
    double dOriginXOffset; //地图左下角x偏移，由此可计算出地图(0,0)点位置，单位m
    double dOriginYOffset; //地图左下角y偏移
    double dTheta; //地图绕(0,0)点逆时针旋转角度（暂时作用是UI调整地图后存储的角度，0度则地图不进行旋转变换）
    uint8_t *pCostMapData; //地图代价数据,0为空，100占用，255未知
    
    //拓扑地图信息
    std::map<int, std::list<TPath> > mapadjMap;//临街表描述的地图路线信息，key:站点ID
    std::map<int, TStation> mapStation; //站点信息,原点在图像中心

}TMapInfo;

//地图文件中的地图数据定义
#define MAP_EMPTY '0'
#define MAP_UNKNOWN '1' //未知
#define MAP_OCCYPY '2'  //占用



//AGV 基本信息

typedef struct BaseInfo
{
    std::string strAGVID;  //AGV编号
    std::string strAGVIP;   //AGV ip地址
    //EConnectStatus eStatus; //AGV连接状态
    int iElectricity;  //电量 %
    double dConfidence;  //置信度 0~1
    //int iPing;  //网络延迟 ms
    bool bIsSync;  //地图及数据是否是同步的
    std::string strMapName; //存储的地图名称
    std::string strComment; //备注

    //AGV控制器信息
    int iRAM; //内存占用百分比
    int iCPU; //CPU占用
    std::string strVersion; //控制器程序版本号

    //其他信息
    double dLineSpeed; //线速度 m/s
    double dAngularSpeed; //角速度 rad/s
    TPointd tPt; //当前坐标
    double dAngle;//车头朝向角(° -PI~PI)
    double dLastAngle; //上一次的角度（默认-90度）
    std::vector<TPointd> vecDWALocalPath;// dwa实时局部路径
}TBaseInfo;


class CAGVMapCtrl;
class CAgvRosCtrl
{
    public:
        CAgvRosCtrl(ros::NodeHandle *nh, st_agv_ctrl *pt_agv_ctrl);
        ~CAgvRosCtrl();
        ros::NodeHandle *ros_nh;
        ros::Subscriber ros_sub_base_info;
        ros::Subscriber ros_sub_map;
        tf::TransformListener *tf_listener;
        ros::Publisher m_pubGridMap; //发布网格地图到“map”队列
        ros::Publisher m_pubGlobalPath; //发布全局路径到"agv_global_path"
        
        int mani_watch_dog;
        float mani_linear_vel;
        float mani_angular_vel;
        float mani_linear_vel_def;
        float mani_angular_vel_def;
        ros::Publisher m_pubManiVel;
        ros::ServiceClient rosClientChassisMode;
        bool RequestChassisMode(uint8_t mode);
        void PubManiCmdVel(float linear_vel, float angular_vel);
        void ManiWatchDogReset();
        void SetChassisVel(float linear_vel, float angular_vel);

        E_AGV_CTRL_STATE e_state;
        void ChangeStateToSlam();
        void ChangeStateToNavi();
        void ChangeStateToError();

    public:
        pthread_rwlock_t rwlock_base;
        TBaseInfo tBaseInfo;
        pthread_rwlock_t rwlock_map;
        TMapInfo tMapInfo;

        bool MapValid;

        CAGVMapCtrl *m_pAGVMapCtrl;//地图对象
        st_agv_ctrl *m_pt_agv_ctrl;

    //slam (cartographer) section
    public:
        ros::ServiceClient rosClientCM;
        ros::ServiceClient rosClientSP;
        //int SlamState;
        int CallChangeModeSvr(bool arg);
        int CallStopPubMapSvr(bool arg);

    //move base section
    public:
        MoveBaseClient *pMoveBaseAc;
        MoveBaseClient_GlobalPath *m_pMoveBase_GlobalPathAc;
        int MoveBaseState;
        st_tcp_connect_info *ptMoveCmdFrom;
        int CallMoveToTarget(st_tcp_connect_info *ptConnect, double x, double y, double w);
        int CallMoveToTarget(st_tcp_connect_info *ptConnectInfo, double dRotateAngle, double dEndAngle,std::vector<TPointd>& global_path);
        int CallCancalTarget();
    public:
   //Base Info Static Data Lock Method---------------------------------------
        void inline WLockBase()
        {
            //pthread_rwlock_wrlock(&rwlock_base);
        };

        void inline RLockBase()
        {
            //pthread_rwlock_rdlock(&rwlock_base);
        };

        void inline ULockBase()
        {
            //pthread_rwlock_unlock(&rwlock_base);
        };

    //Map Info Static Data Lock Method---------------------------------------
        void inline WLockMap()
        {
            pthread_rwlock_wrlock(&rwlock_map);
        };

        void inline RLockMap()
        {
            pthread_rwlock_rdlock(&rwlock_map);
        };

        void inline ULockMap()
        {
            pthread_rwlock_unlock(&rwlock_map);
        };

    
    //Ros Function
    public:
        int Init();
        void SetupSubscription();
        void agvPosSubCB();
        void agvMapSubCB(const nav_msgs::OccupancyGridConstPtr& msg);
        int agvPosTransform();

        //发布网格地图
        void PublishGridmap(); 
    
};




//int agv_ros_ctrl_init(st_agv_ros_ctrl *pt_ros_ctrl);

#endif // AGV_ROS_CTRL_H