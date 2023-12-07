#ifndef AGV_MAP_CTRL_H
#define AGV_MAP_CTRL_H
#include <string>
#include <map>
#include <list>
#include <vector>
#include "agv_type.h"

//像素距离,认为浮点数相等的阈值范围
#define EPS_0 1.0e-6   

//像素距离
#define TOLERANCE_PT_ON_STATION 5

//地图文件中的地图数据定义
#define MAP_EMPTY '0'
#define MAP_UNKNOWN '1' //未知
#define MAP_OCCYPY '2'  //占用


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
    uint8_t pCostMapData[16 * 1024 * 1024]; //地图代价数据,0为空，100占用，255未知
    
    //拓扑地图信息
    std::map<int, std::list<TPath> > mapadjMap;//临街表描述的地图路线信息，key:站点ID
    std::map<int, TStation> mapStation; //站点信息,原点在图像中心

}TMapInfo;



//A星算法所需站点信息
typedef struct AStarStation
{
    int iStationID; //站点编号       可获取坐标信息
    double s_g;    // 起点到此点的距离     实际成本代价
    double s_f;    // 预测的此点到终点的距离       暂定直线距离
    int fStationID;   // 父站点ID
    int TPathID;      //父站点到本站点的路径ID
    double Lengh_TPathID;                 //路径ID对应的路径长度
}TAStarStation;


class CAGVMapCtrl
{
public:
    CAGVMapCtrl(AgvCtrl *pt_agv_ctrl);
    ~CAGVMapCtrl();

    /**
     * @brief 初始化，加载地图
     * 
     * @return int 
     */
    int Init();

    /**
     * @brief : 解析地图json数据，保存到本地，转换拓扑数据及代价地图数据
     * 
     * @param pfile_buffer : 地图json文件流
     * @return int 
     */
    int save_map(uint8_t *pfile_buffer);

    /**
     * @brief Get the Path object
     * 
     * @param tAGVPos agv 当前位置（world坐标系）
     * @param iDesStation 目标站点ID (查出的坐标是像素坐标，(0,0在中心位置))
     * @param dRotateToAngle 起始点AGV需要旋转的角度（-PI ~ PI）
     * @return std::vector<TPointd> 返回网格路径信息（world坐标系） 
     */
    std::vector<TPointd> get_path(TPointd tAGVPos, int iDesStation, double &dRotateToAngle);

    /**
     * @brief Get the grid map object
     * 
     * @return nav_msgs::OccupancyGrid
     */
    //nav_msgs::OccupancyGrid &get_grid_map(){return m_advGridMap;}

    /**
     * @brief 图像坐标转世界坐标   图像原点在中心位置
     * 
     * @param  dxMap  dyMap 图像点x，y ； dx dy 世界坐标系点坐标值
     * @return  
     */
    void MapToWorld(double dxMap, double dyMap, double &dx, double &dy);

    /**
     * @brief 世界坐标转图像坐标   图像原点在中心位置
     * 
     * @param   dx dy 世界坐标系点坐标值  dxMap  dyMap 图像点x，y    
     * @return  bool
     */
    bool WorldToMap(double dx, double dy, double &dxMap, double &dyMap);

    void test();

    void test_get_start_end_station(TStation &tStart, TStation &tEnd){
        tStart = m_tStartStation;
        tEnd = m_tEndStation;
    };

private:
    /**
     * @brief 解析json文件到TMapInfo
     * 
     * @param pfile_buffer 
     * @return int 
     */
    int parse_json(uint8_t *pfile_buffer);

    void to_grid_map();

    /**
     * @brief 三阶贝塞尔曲线表达式求解
     * 
     * @param  t  p
     * @return  TPointd
     */
    TPointd get_point_Bezier(double t,TPointd *p);

    /**
     * @brief 路径ID段长度计算
     * 
     * @param  t_path
     * @return  double
     */
    double path_length(TPath t_path);

    /**
     * @brief 两站点间直线距离计算
     * 
     * @param  station1_ID    station2_ID
     * @return  double
     */
    double station_line_length(int station1_ID,int station2_ID);

    /**
     * @brief 查找给定站点所有相邻站点,根据不同情况决定是否放入openlist中
     * 忽略已在closelist中的站点,已在openlist中的相邻站点,需判断是否需要更新父站点
     * @param  ID_station需更新的站点  iDesStation目标站点，计算用
     * @return  
     */
    void update_openlist(int ID_station,int iDesStation);

    /**
     * @brief 查找openlist中找到F值最小的站点
     * 
     * @param  
     * @return  int
     */
    int find_select_station();

    /**
     * @brief 两点间直线插值
     * 
     * @param  start  end
     * @return  vector<TPointd>
     */
    std::vector<TPointd> line_path_interpolation(TPointd start,TPointd end);

    /**
     * @brief 路径ID段路径插值
     * 
     * @param  t_path end_stationID 规划终点ID  Lengh_TPathID  该段路径长度
     * @return  vector<TPointd>
     */
    std::vector<TPointd> ID_path_interpolation(TPath t_path,int end_stationID,double Lengh_TPathID);

    

    /**
         * @brief 计算两个点的夹角，返回的范围是-PI~PI
         * 
         * @param x0 起始
         * @param y0 
         * @param x1 终止
         * @param y1 
         * @return double 返回弧度 -PI~PI
         */
        double TwoPointAngle(double dx1, double dy1, double dx2, double dy2);

    //v2.0地图信息
public:
    TMapInfo m_tMapInfo; 

private:
    AgvCtrl *m_pt_agv_ctrl;
    std::map<int, TAStarStation >  Openlist; 
    std::map<int, TAStarStation >  Closelist; 
    TAStarStation m_tAStartStation; 
    //要发布的网格地图
    //nav_msgs::OccupancyGrid m_advGridMap;

    //测试使用---
    //起始站点
    TStation m_tStartStation;
    //终点站信息
    TStation m_tEndStation; 
    
};


#endif // AGV_FILE_SVR_H