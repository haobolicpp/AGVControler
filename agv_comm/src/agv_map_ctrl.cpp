/*
【A*基本原理】
利用节点的评估函数f = g + h，g为当前节点到起点的距离(或时间)代价，h为当前节点到终点的距离
h一般用曼哈顿距离来简单计算
将待检测的节点的相邻节点计算完代价信息以及父节点后，放到open表中，而已经检测过的节点放到close表中
搜寻点时每次从open表中找到代价最小的点，计算它的连接点信息后，将该点放入close表中
退出条件：1、当搜索过程中open表为空，此时可能出现了跨地图的情况，终点在另一个不连接的地图中
2、简化期间，搜索到的相邻节点如果是终点，则停止搜索
【注意】1、往open表中添加的节点可能已经在open表中了，这时需要比较要添加的节点和open表中已有节点的代价
只有要添加的节点的代价<open表中节点代价时，才能将该点加入open表
*/
#include <stdio.h>
#include <pthread.h>
#include <mqueue.h>
#include <errno.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/uio.h>
#include <net/if.h>
#include <sys/select.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fstream>

#include "ros/ros.h"
#include "agv_comm/agv_type.h"
#include "agv_comm/agv_cmd.hpp"
#include "agv_comm/agv_ctrl.hpp"
#include "agv_comm/agv_tcp_ctrl.hpp"
#include "agv_comm/agv_tcp_svr.hpp"
#include "agv_comm/agv_file_svr.hpp"
#include "agv_comm/agv_base_svr.hpp"
#include "agv_comm/agv_map_ctrl.h"
#include "agv_comm/agv_config.h"
#include "agv_comm/agv_type.h"
#include "agv_comm/json.hpp"

using json = nlohmann::json;

CAGVMapCtrl::CAGVMapCtrl(st_agv_ctrl *pt_agv_ctrl)
{
    m_pt_agv_ctrl = pt_agv_ctrl;
    m_tMapInfo.pCostMapData = nullptr;
}

CAGVMapCtrl::~CAGVMapCtrl()
{
    if (m_tMapInfo.pCostMapData != nullptr)
    {
        free(m_tMapInfo.pCostMapData);
    }
}

int CAGVMapCtrl::init()
{
    FILE *pf = nullptr;
    char str_file[512] = {0};
    int ifile_len = 0;
    uint8_t *pfile_buffer = nullptr;

    sprintf(str_file, "/home/%s/Downloads/map/%s.json", getlogin(), m_pt_agv_ctrl->t_agv_base_config.chMapName);
    pf = fopen(str_file, "r");
    if (pf == nullptr)
    {
        printf("map:%s.json is not existed!\n", m_pt_agv_ctrl->t_agv_base_config.chMapName);
        return -1;
    }
    if (-1 == fseek(pf, 0, SEEK_END))
    {
        return -1;
    }
    if (-1 == (ifile_len=ftell(pf)))
    {
        return -1;
    }
    if (-1 == fseek(pf, 0, SEEK_SET))
    {
        return -1;
    }
    pfile_buffer = (uint8_t*)malloc(ifile_len+1);
    if (pfile_buffer == nullptr)
    {
        return -1;
    }
    memset(pfile_buffer, 0, ifile_len);
    if (1 != fread(pfile_buffer, ifile_len, 1, pf))
    {
        return -1;
    }
    fclose(pf);
    pfile_buffer[ifile_len] = '\0';

    //解析json
    if (0 != parse_json(pfile_buffer))
    {
        return -1;
    }

    free(pfile_buffer);

    //转换发布地图
    to_grid_map();
    
    return 0;
}

int CAGVMapCtrl::save_map(uint8_t *pfile_buffer)
{  
    FILE *pfl = nullptr;
    char strDir[512] = {0};
    char strFile[512] = {0};
    int ifile_len = strlen((const char*)pfile_buffer);

    //文件目录
    sprintf(strDir, "/home/%s/Downloads/map", getlogin());

    //解析json
    if (0 != parse_json(pfile_buffer))
    {
        return -1;
    }
    
    //保存地图
    if (0 != create_dir_recursive(strDir))
    {
        return -1;
    }
    char strMapPath[512] = {0};
    sprintf(strMapPath, "%s/%s.json", strDir, m_tMapInfo.strMapName.c_str());
    pfl = fopen(strMapPath, "w+");
    if (NULL == pfl)
    {
        printf("fopen error:%d\n", errno);
        fclose(pfl);
        return -1;
    }
    if (1 != fwrite(pfile_buffer, ifile_len, 1, pfl))
    {
        printf("save_map fwrite map error:%d\n", errno);
        return -1;
    }
    fflush(pfl);
    fclose(pfl);

    //修改AGV基础配置信息
    memset(m_pt_agv_ctrl->t_agv_base_config.chMapName, 0, sizeof(m_pt_agv_ctrl->t_agv_base_config.chMapName));
    memset(m_pt_agv_ctrl->t_agv_base_config.chMapMD5, 0, sizeof(m_pt_agv_ctrl->t_agv_base_config.chMapMD5));
    strncpy((char*)m_pt_agv_ctrl->t_agv_base_config.chMapName, m_tMapInfo.strMapName.c_str(), sizeof(m_pt_agv_ctrl->t_agv_base_config.chMapName));
    //strncpy(m_pt_agv_ctrl->t_agv_base_config.chMapMD5, );
    if (0 != write_agv_config_to_json(m_pt_agv_ctrl))
    {
        return -1;
    }

    //转换发布地图
    to_grid_map();
    
    return 0;
}

void CAGVMapCtrl::to_grid_map()
{
    ros::Time::waitForValid();
    m_advGridMap.info.map_load_time = ros::Time::now();
    m_advGridMap.header.frame_id = std::string("map");
    m_advGridMap.header.stamp = ros::Time::now();

    m_advGridMap.info.width = m_tMapInfo.iWidth;
    m_advGridMap.info.height = m_tMapInfo.iHeight;
    m_advGridMap.info.resolution = m_tMapInfo.dResolution;
    m_advGridMap.info.origin.position.x = m_tMapInfo.dOriginXOffset;
    m_advGridMap.info.origin.position.y = m_tMapInfo.dOriginYOffset;
    m_advGridMap.info.origin.position.z = 0.0;

    m_advGridMap.info.origin.orientation.x = 0;
    m_advGridMap.info.origin.orientation.y = 0;
    m_advGridMap.info.origin.orientation.z = 0;
    m_advGridMap.info.origin.orientation.w = 0;

    // Allocate space to hold the data
    m_advGridMap.data.resize(m_advGridMap.info.width * m_advGridMap.info.height);
    memcpy((void*)&m_advGridMap.data[0], m_tMapInfo.pCostMapData, m_advGridMap.info.width * m_advGridMap.info.height);
}

int CAGVMapCtrl::parse_json(uint8_t *pfile_buffer)
{
    //printf("%s\n",pfile_buffer);
    //解析json
    try
    {
        if (!json::accept(pfile_buffer))
    {
        return -1;
    }
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    
    
    auto itValue = json::parse(pfile_buffer);
    //解析地图头
    auto itmap_header = itValue.value("map_header", json::object());
    m_tMapInfo.strMapName = itmap_header.value("map_name", "default");
    m_tMapInfo.strVersion = itmap_header.value("ver", "1.0.0");
    m_tMapInfo.dResolution = itmap_header.value("resolution", 0.05);
    m_tMapInfo.iWidth = itmap_header.value("width", 100);
    m_tMapInfo.iHeight = itmap_header.value("height", 100);
    m_tMapInfo.dOriginXOffset = itmap_header.value("originX", -5.0);
    m_tMapInfo.dOriginYOffset = itmap_header.value("originY", -5.0);

    //解析地图数据(CostMap2D)
    free(m_tMapInfo.pCostMapData);
    m_tMapInfo.pCostMapData = (uint8_t*)malloc(m_tMapInfo.iHeight*m_tMapInfo.iWidth);
    auto itmap_data = itValue.value("map_data", "");
    int index = 0;  
    for (auto &itV : itmap_data)
    {
        if (itV == MAP_EMPTY)
        {
            m_tMapInfo.pCostMapData[index] = 0;
        }
        else if (itV == MAP_UNKNOWN)
        {
            m_tMapInfo.pCostMapData[index] = 255;
        }
        else if (itV == MAP_OCCYPY)
        {
            m_tMapInfo.pCostMapData[index] = 100;
        }else{}
        index++;
    }

    //解析站点
    m_tMapInfo.mapadjMap.clear();
    m_tMapInfo.mapStation.clear();
    auto itmap_station = itValue.value("station_data", json::array());
    for (auto &itValue : itmap_station)
    {
        TStation tStation;
        tStation.iStationID = itValue.value("id", 0);
        tStation.eType = (EStationType)itValue.value("type", 0);
        auto it_pos = itValue.value("pos", json::object());
        tStation.tPt.dX = it_pos.value("x", 0.0);
        tStation.tPt.dY = it_pos.value("y", 0.0);
        tStation.dAngle = it_pos.value("angle", 0);
        tStation.strComment = itValue.value("comment", "");

        m_tMapInfo.mapadjMap.insert(std::make_pair(tStation.iStationID, std::list<TPath>()));
        m_tMapInfo.mapStation.insert(std::make_pair(tStation.iStationID, tStation));
    }
    //解析路线
    auto itmap_path = itValue.value("path_data", json::array());
    for (auto &itValue : itmap_path)
    {
        TPath tPath;
        tPath.iPathID = itValue.value("id", 0);
        tPath.eType = (EPathType)itValue.value("type", 0);
        tPath.iStartStation = itValue.value("start_station", 0);
        tPath.iEndStation = itValue.value("end_station", 0);
        tPath.eDirect = (EPathDirect)itValue.value("direct", 0);
        if (tPath.eType == EPathType::PathType_Line)
        {
        }
        else if(tPath.eType == EPathType::PathType_Circle)
        {
            auto it_center = itValue.value("circle_center", json::object());
            tPath.udata.tArcData.tPtCenter.dX = it_center.value("x", 0.0);
            tPath.udata.tArcData.tPtCenter.dY = it_center.value("y", 0.0);
            tPath.udata.tArcData.bIsGood = itValue.value("isgood", 0);
        }
        else if (tPath.eType == EPathType::PathType_Bezier)
        {
            auto it_start_control = itValue.value("start_control_pt", json::object());
            auto it_end_control = itValue.value("end_control_pt", json::object());
            tPath.udata.tBazierData.tPtNearStart.dX = it_start_control.value("x", 0.0);
            tPath.udata.tBazierData.tPtNearStart.dY = it_start_control.value("y", 0.0);
            tPath.udata.tBazierData.tPtNearEnd.dX = it_end_control.value("x", 0.0);
            tPath.udata.tBazierData.tPtNearEnd.dY = it_end_control.value("y", 0.0);
        }else{}

        auto itStartNode = m_tMapInfo.mapadjMap.find(tPath.iStartStation);
        if (itStartNode != m_tMapInfo.mapadjMap.end())
        {
            itStartNode->second.push_back(tPath);
        }
        auto itEndNode = m_tMapInfo.mapadjMap.find(tPath.iEndStation);
        if (itEndNode != m_tMapInfo.mapadjMap.end())
        {
            itEndNode->second.push_back(tPath);
        }
    }
    return 0;
}

void CAGVMapCtrl::test()
{
    std::vector<TPointd>  paths;
    //创建地图
    std::list<TPath> path_list; 
    TStation station;
    TPointd tAGVPos;
    m_tMapInfo.mapadjMap.clear();
    m_tMapInfo.mapStation.clear();

//参数设置
    m_tMapInfo.iHeight = 1000;
    m_tMapInfo.iWidth = 1000;
    m_tMapInfo.dResolution = 0.2;          //m/像素
    m_tMapInfo.dOriginXOffset = -100.0;
    m_tMapInfo.dOriginYOffset = -200.0;

//站点设置
//s1
    station.iStationID = 1;
    station.tPt.dX = -2.0*m_tMapInfo.iWidth/5.0;
    station.tPt.dY = -2.0*m_tMapInfo.iHeight/5.0;
    m_tMapInfo.mapStation[station.iStationID] = station;
//s2
    station.iStationID = 2;
    station.tPt.dX = -m_tMapInfo.iWidth/5.0;
    station.tPt.dY = -m_tMapInfo.iHeight/10.0;
    m_tMapInfo.mapStation[station.iStationID] = station;
//s3
    station.iStationID = 3;
    station.tPt.dX = 0.0;
    station.tPt.dY = -3.0*m_tMapInfo.iHeight/10.0;
    m_tMapInfo.mapStation[station.iStationID] = station;
//s4
    station.iStationID = 4;
    station.tPt.dX = 9.0*m_tMapInfo.iWidth/20.0;
    station.tPt.dY = -9.0*m_tMapInfo.iHeight/20.0;
    m_tMapInfo.mapStation[station.iStationID] = station;
//s5
    station.iStationID = 5;
    station.tPt.dX = m_tMapInfo.iWidth/5.0;
    station.tPt.dY = -m_tMapInfo.iHeight/6.0;
    m_tMapInfo.mapStation[station.iStationID] = station;
//s6
    station.iStationID = 6;
    station.tPt.dX = 9.0*m_tMapInfo.iWidth/20.0;
    station.tPt.dY = -m_tMapInfo.iHeight/6.0;
    m_tMapInfo.mapStation[station.iStationID] = station;
//s7
    station.iStationID = 7;
    station.tPt.dX = 9.0*m_tMapInfo.iWidth/20.0;
    station.tPt.dY = m_tMapInfo.iHeight/6.0;
    m_tMapInfo.mapStation[station.iStationID] = station;
//s8
    station.iStationID = 8;
    station.tPt.dX = m_tMapInfo.iWidth/5.0;
    station.tPt.dY = m_tMapInfo.iHeight/12.0;
    m_tMapInfo.mapStation[station.iStationID] = station;
//s9
    station.iStationID = 9;
    station.tPt.dX = 0.0;
    station.tPt.dY = m_tMapInfo.iHeight/5.0;
    m_tMapInfo.mapStation[station.iStationID] = station;
//s10
    station.iStationID = 10;
    station.tPt.dX = -m_tMapInfo.iWidth/5.0;
    station.tPt.dY = 0.0;
    m_tMapInfo.mapStation[station.iStationID] = station;
//s11
    station.iStationID = 11;
    station.tPt.dX = -2.0*m_tMapInfo.iWidth/5.0;
    station.tPt.dY = m_tMapInfo.iHeight/5.0;
    m_tMapInfo.mapStation[station.iStationID] = station;
//s12
    station.iStationID = 12;
    station.tPt.dX = -9.0*m_tMapInfo.iWidth/20.0;
    station.tPt.dY = 9.0*m_tMapInfo.iHeight/20.0;
    m_tMapInfo.mapStation[station.iStationID] = station;
//s13
    station.iStationID = 13;
    station.tPt.dX = m_tMapInfo.iWidth/5.0;
    station.tPt.dY = 9.0*m_tMapInfo.iHeight/20.0;
    m_tMapInfo.mapStation[station.iStationID] = station;

//路径设置
    TPath path_1;
    path_1.iPathID = 1;
    path_1.eType = EPathType::PathType_Bezier;
    path_1.udata.tBazierData.tPtNearStart.dX = -2.0*m_tMapInfo.iWidth/5.0;
    path_1.udata.tBazierData.tPtNearStart.dY = -3.0*m_tMapInfo.iHeight/10.0;
    path_1.udata.tBazierData.tPtNearEnd.dX = -m_tMapInfo.iWidth/5.0;
    path_1.udata.tBazierData.tPtNearEnd.dY = -m_tMapInfo.iHeight/6.0;
    path_1.iStartStation = 1;
    path_1.iEndStation = 2;
    path_1.eDirect = EPathDirect::PathDirect_Bothway;
    TPath path_2;
    path_2.iPathID = 2;
    path_2.eType = EPathType::PathType_Bezier;
    path_2.udata.tBazierData.tPtNearStart.dX = -m_tMapInfo.iWidth/5.0;
    path_2.udata.tBazierData.tPtNearStart.dY = -m_tMapInfo.iHeight/6.0;
    path_2.udata.tBazierData.tPtNearEnd.dX = -m_tMapInfo.iWidth/10.0;
    path_2.udata.tBazierData.tPtNearEnd.dY = -3.0*m_tMapInfo.iHeight/10.0;
    path_2.iStartStation = 2;
    path_2.iEndStation = 3;
    path_2.eDirect = EPathDirect::PathDirect_Bothway;
    TPath path_3;
    path_3.iPathID = 3;
    path_3.eType = EPathType::PathType_Bezier;
    path_3.udata.tBazierData.tPtNearStart.dX = -m_tMapInfo.iWidth/5.0;
    path_3.udata.tBazierData.tPtNearStart.dY = -2.0*m_tMapInfo.iHeight/5.0;
    path_3.udata.tBazierData.tPtNearEnd.dX = -m_tMapInfo.iWidth/10.0;
    path_3.udata.tBazierData.tPtNearEnd.dY = -3.0*m_tMapInfo.iHeight/10.0;
    path_3.iStartStation = 1;
    path_3.iEndStation = 3;
    path_3.eDirect = EPathDirect::PathDirect_Bothway;
    TPath path_4;
    path_4.iPathID = 4;
    path_4.eType = EPathType::PathType_Bezier;
    path_4.udata.tBazierData.tPtNearStart.dX = -2.0*m_tMapInfo.iWidth/5.0;
    path_4.udata.tBazierData.tPtNearStart.dY = -3.0*m_tMapInfo.iHeight/10.0;
    path_4.udata.tBazierData.tPtNearEnd.dX =-3.0*m_tMapInfo.iWidth/10.0;
    path_4.udata.tBazierData.tPtNearEnd.dY = m_tMapInfo.iHeight/10.0;
    path_4.iStartStation = 1;
    path_4.iEndStation = 11;
    path_4.eDirect = EPathDirect::PathDirect_Bothway;
    TPath path_5;
    path_5.iPathID = 5;
    path_5.eType = EPathType::PathType_Bezier;
    path_5.udata.tBazierData.tPtNearStart.dX = -m_tMapInfo.iWidth/5.0;
    path_5.udata.tBazierData.tPtNearStart.dY = 0.0;
    path_5.udata.tBazierData.tPtNearEnd.dX =0.0;
    path_5.udata.tBazierData.tPtNearEnd.dY = m_tMapInfo.iHeight/6.0;
    path_5.iStartStation = 2;
    path_5.iEndStation = 9;
    path_5.eDirect = EPathDirect::PathDirect_StartToEnd;
    TPath path_6;
    path_6.iPathID = 6;
    path_6.eType = EPathType::PathType_Bezier;
    path_6.udata.tBazierData.tPtNearStart.dX = 0.0;
    path_6.udata.tBazierData.tPtNearStart.dY = -m_tMapInfo.iHeight/5.0;
    path_6.udata.tBazierData.tPtNearEnd.dX = m_tMapInfo.iWidth/6.0;
    path_6.udata.tBazierData.tPtNearEnd.dY = -m_tMapInfo.iHeight/6.0;
    path_6.iStartStation = 3;
    path_6.iEndStation = 5;
    path_6.eDirect = EPathDirect::PathDirect_StartToEnd;
    TPath path_7;
    path_7.iPathID = 7;
    path_7.eType = EPathType::PathType_Bezier;
    path_7.udata.tBazierData.tPtNearStart.dX = 0.0;
    path_7.udata.tBazierData.tPtNearStart.dY = -2.0*m_tMapInfo.iHeight/5.0;
    path_7.udata.tBazierData.tPtNearEnd.dX = 2.0*m_tMapInfo.iWidth/5.0;
    path_7.udata.tBazierData.tPtNearEnd.dY = -9.0*m_tMapInfo.iHeight/20.0;
    path_7.iStartStation = 3;
    path_7.iEndStation = 4;
    path_7.eDirect = EPathDirect::PathDirect_EndToStart;
    TPath path_8;
    path_8.iPathID = 8;
    path_8.eType = EPathType::PathType_Bezier;
    path_8.udata.tBazierData.tPtNearStart.dX = 9.0*m_tMapInfo.iWidth/20.0;
    path_8.udata.tBazierData.tPtNearStart.dY = -2.0*m_tMapInfo.iHeight/5.0;
    path_8.udata.tBazierData.tPtNearEnd.dX = m_tMapInfo.iWidth/5.0;
    path_8.udata.tBazierData.tPtNearEnd.dY = -m_tMapInfo.iHeight/5.0;
    path_8.iStartStation = 4;
    path_8.iEndStation = 5;
    path_8.eDirect = EPathDirect::PathDirect_EndToStart;
    TPath path_9;
    path_9.iPathID = 9;
    path_9.eType = EPathType::PathType_Bezier;
    path_9.udata.tBazierData.tPtNearStart.dX = m_tMapInfo.iWidth/2.0;
    path_9.udata.tBazierData.tPtNearStart.dY = -2.0*m_tMapInfo.iHeight/5.0;
    path_9.udata.tBazierData.tPtNearEnd.dX = 2.0*m_tMapInfo.iWidth/5.0;
    path_9.udata.tBazierData.tPtNearEnd.dY = -m_tMapInfo.iHeight/5.0;
    path_9.iStartStation = 4;
    path_9.iEndStation = 6;
    path_9.eDirect = EPathDirect::PathDirect_Bothway;
    TPath path_10;
    path_10.iPathID = 10;
    path_10.eType = EPathType::PathType_Line;
    path_10.udata.tBazierData.tPtNearStart.dX = m_tMapInfo.iWidth/6.0;
    path_10.udata.tBazierData.tPtNearStart.dY = -m_tMapInfo.iHeight/6.0;
    path_10.udata.tBazierData.tPtNearEnd.dX = m_tMapInfo.iWidth/6.0;
    path_10.udata.tBazierData.tPtNearEnd.dY = 0.0;
    path_10.iStartStation = 5;
    path_10.iEndStation = 8;
    path_10.eDirect = EPathDirect::PathDirect_Bothway;
    TPath path_11;
    path_11.iPathID = 11;
    path_11.eType = EPathType::PathType_Bezier;
    path_11.udata.tBazierData.tPtNearStart.dX = 0.0;
    path_11.udata.tBazierData.tPtNearStart.dY = m_tMapInfo.iHeight/6.0;
    path_11.udata.tBazierData.tPtNearEnd.dX = 2.0*m_tMapInfo.iWidth/5.0;
    path_11.udata.tBazierData.tPtNearEnd.dY = -m_tMapInfo.iHeight/5.0;
    path_11.iStartStation = 9;
    path_11.iEndStation = 6;
    path_11.eDirect = EPathDirect::PathDirect_Bothway;
    TPath path_12;
    path_12.iPathID = 12;
    path_12.eType = EPathType::PathType_Bezier;
    path_12.udata.tBazierData.tPtNearStart.dX = 2.0*m_tMapInfo.iWidth/5.0;
    path_12.udata.tBazierData.tPtNearStart.dY = -m_tMapInfo.iHeight/7.0;
    path_12.udata.tBazierData.tPtNearEnd.dX = 2.0*m_tMapInfo.iWidth/5.0;
    path_12.udata.tBazierData.tPtNearEnd.dY = m_tMapInfo.iHeight/7.0;
    path_12.iStartStation = 6;
    path_12.iEndStation = 7;
    path_12.eDirect = EPathDirect::PathDirect_StartToEnd;
    TPath path_13;
    path_13.iPathID = 13;
    path_13.eType = EPathType::PathType_Bezier;
    path_13.udata.tBazierData.tPtNearStart.dX = 9.0*m_tMapInfo.iWidth/20.0;
    path_13.udata.tBazierData.tPtNearStart.dY = m_tMapInfo.iHeight/7.0;
    path_13.udata.tBazierData.tPtNearEnd.dX = m_tMapInfo.iWidth/5.0;
    path_13.udata.tBazierData.tPtNearEnd.dY = m_tMapInfo.iHeight/7.0;
    path_13.iStartStation = 7;
    path_13.iEndStation = 8;
    path_13.eDirect = EPathDirect::PathDirect_Bothway;
    TPath path_14;
    path_14.iPathID = 14;
    path_14.eType = EPathType::PathType_Bezier;
    path_14.udata.tBazierData.tPtNearStart.dX = 9.0*m_tMapInfo.iWidth/20.0;
    path_14.udata.tBazierData.tPtNearStart.dY = m_tMapInfo.iHeight/5.0;
    path_14.udata.tBazierData.tPtNearEnd.dX = m_tMapInfo.iWidth/5.0;
    path_14.udata.tBazierData.tPtNearEnd.dY = m_tMapInfo.iHeight/7.0;
    path_14.iStartStation = 7;
    path_14.iEndStation = 9;
    path_14.eDirect = EPathDirect::PathDirect_Bothway;
    TPath path_15;
    path_15.iPathID = 15;
    path_15.eType = EPathType::PathType_Bezier;
    path_15.udata.tBazierData.tPtNearStart.dX = m_tMapInfo.iWidth/6.0;
    path_15.udata.tBazierData.tPtNearStart.dY = 0.0;
    path_15.udata.tBazierData.tPtNearEnd.dX = 0.0;
    path_15.udata.tBazierData.tPtNearEnd.dY = 0.0;
    path_15.iStartStation = 8;
    path_15.iEndStation = 10;
    path_15.eDirect = EPathDirect::PathDirect_Bothway;
    TPath path_16;
    path_16.iPathID = 16;
    path_16.eType = EPathType::PathType_Bezier;
    path_16.udata.tBazierData.tPtNearStart.dX = -m_tMapInfo.iWidth/5.0;
    path_16.udata.tBazierData.tPtNearStart.dY = m_tMapInfo.iHeight/10.0;
    path_16.udata.tBazierData.tPtNearEnd.dX = -3.0*m_tMapInfo.iWidth/10.0;
    path_16.udata.tBazierData.tPtNearEnd.dY = m_tMapInfo.iHeight/10.0;
    path_16.iStartStation = 10;
    path_16.iEndStation = 11;
    path_16.eDirect = EPathDirect::PathDirect_Bothway;
    TPath path_17;
    path_17.iPathID = 17;
    path_17.eType = EPathType::PathType_Bezier;
    path_17.udata.tBazierData.tPtNearStart.dX = -9.0*m_tMapInfo.iWidth/20.0;
    path_17.udata.tBazierData.tPtNearStart.dY = 2.0*m_tMapInfo.iHeight/5.0;
    path_17.udata.tBazierData.tPtNearEnd.dX = 0.0;
    path_17.udata.tBazierData.tPtNearEnd.dY = 2.0*m_tMapInfo.iHeight/5.0;
    path_17.iStartStation = 12;
    path_17.iEndStation = 9;
    path_17.eDirect = EPathDirect::PathDirect_Bothway;
    TPath path_18;
    path_18.iPathID = 18;
    path_18.eType = EPathType::PathType_Bezier;
    path_18.udata.tBazierData.tPtNearStart.dX = -2.0*m_tMapInfo.iWidth/5.0;
    path_18.udata.tBazierData.tPtNearStart.dY = 3.0*m_tMapInfo.iHeight/10.0;
    path_18.udata.tBazierData.tPtNearEnd.dX = -9.0*m_tMapInfo.iWidth/20.0;
    path_18.udata.tBazierData.tPtNearEnd.dY = 2.0*m_tMapInfo.iHeight/5.0;
    path_18.iStartStation = 11;
    path_18.iEndStation = 12;
    path_18.eDirect = EPathDirect::PathDirect_Bothway;
 
//s1
    path_list.clear();
    path_list.push_back (path_1);
    path_list.push_back (path_3);
   // path_list.push_back (path_4);
    m_tMapInfo.mapadjMap[1] = path_list;
    path_list.clear();
//s2
    path_list.push_back (path_1);
    path_list.push_back (path_2);
    //path_list.push_back (path_5);
    m_tMapInfo.mapadjMap[2] = path_list;
    path_list.clear();
//s3
    path_list.push_back (path_2);
    path_list.push_back (path_3);
    path_list.push_back (path_6);
    path_list.push_back (path_7);
    m_tMapInfo.mapadjMap[3] = path_list;
    path_list.clear();
//s4
    path_list.push_back (path_7);
    path_list.push_back (path_8);
    path_list.push_back (path_9);
    m_tMapInfo.mapadjMap[4] = path_list;
    path_list.clear(); 
//s5
    path_list.push_back (path_6);
    path_list.push_back (path_8);
    path_list.push_back (path_10);
    m_tMapInfo.mapadjMap[5] = path_list;
    path_list.clear(); 
//s6
    path_list.push_back (path_9);
    path_list.push_back (path_11);
    path_list.push_back (path_12);
    m_tMapInfo.mapadjMap[6] = path_list;
    path_list.clear(); 
//s7
    path_list.push_back (path_14);
    path_list.push_back (path_13);
    path_list.push_back (path_12);
    m_tMapInfo.mapadjMap[7] = path_list;
    path_list.clear(); 
//s8
    path_list.push_back (path_10);
    path_list.push_back (path_13);
    path_list.push_back (path_15);
    m_tMapInfo.mapadjMap[8] = path_list;
    path_list.clear(); 
//s9
    path_list.push_back (path_14);
    path_list.push_back (path_17);
    path_list.push_back (path_11);
    //path_list.push_back (path_5);
    m_tMapInfo.mapadjMap[9] = path_list;
    path_list.clear(); 
//s10
    //path_list.push_back (path_16);
    path_list.push_back (path_15);
    m_tMapInfo.mapadjMap[10] = path_list;
    path_list.clear(); 
//s11
    //path_list.push_back (path_4);
    //path_list.push_back (path_16);
    path_list.push_back (path_18);
    m_tMapInfo.mapadjMap[11] = path_list;
    path_list.clear(); 
//s12
    path_list.push_back (path_17);
    path_list.push_back (path_18);
    m_tMapInfo.mapadjMap[12] = path_list;
    path_list.clear(); 

//测试
    tAGVPos.dX = -2.0*m_tMapInfo.iWidth/5.0;
    tAGVPos.dY = m_tMapInfo.iHeight/5.0;
    clock_t startTime,endTime;
    startTime = clock();//计时开始
    //paths = get_path(tAGVPos, 4);
    endTime = clock();//计时结束
    double time = (double)(endTime - startTime) / CLOCKS_PER_SEC;           //s
    int num = paths.size();
}

TPointd CAGVMapCtrl::get_point_Bezier(double t,TPointd *p)
{
    TPointd p_t;

    p_t.dX = ((*p).dX)*(1-t)*(1-t)*(1-t) + 3*((*(p+1)).dX)*t*(1-t)*(1-t) + 3*((*(p+2)).dX)*t*t*(1-t) + ((*(p+3)).dX)*t*t*t;
    p_t.dY = ((*p).dY)*(1-t)*(1-t)*(1-t) + 3*((*(p+1)).dY)*t*(1-t)*(1-t) + 3*((*(p+2)).dY)*t*t*(1-t) + ((*(p+3)).dY)*t*t*t;

    return p_t;
}

double CAGVMapCtrl::path_length(TPath t_path)
{
    double length = 0.0;
    TPointd Piont_Bezier[4];                 //存放4个控制点
    double L_Bezier;            
    double delta_t_Bezier;
    double d_x;
    double d_y;
    TPointd P_last;      
    TPointd P_now;               
    //计算该段路长
    
    switch(t_path.eType)
    {
        case EPathType::PathType_Bezier:                                                        //贝塞尔曲线
        {
            auto it_P = m_tMapInfo.mapStation.find(t_path.iStartStation);  
            Piont_Bezier[0] = it_P->second.tPt;
            Piont_Bezier[1] = t_path.udata.tBazierData.tPtNearStart;
            Piont_Bezier[2] = t_path.udata.tBazierData.tPtNearEnd;
            it_P = m_tMapInfo.mapStation.find(t_path.iEndStation);   
            Piont_Bezier[3] = it_P->second.tPt;
        
        //法1  累加法
        //计算四个控制点连线长度
            for (int i = 0; i < 3; i++)
            {
                d_x = Piont_Bezier[i].dX - Piont_Bezier[i+1].dX;
                d_y = Piont_Bezier[i].dY - Piont_Bezier[i+1].dY;

                length += sqrt(d_x*d_x + d_y*d_y);
            }
            //计算delta_t
            delta_t_Bezier = 1/(10*length);                     //细分越细越精确
            //求和
            P_last = Piont_Bezier[0];                                   //初始化值
            length = 0.0;
            for (double t = delta_t_Bezier; t< 1.0; t+=delta_t_Bezier)      
            {
                P_now = get_point_Bezier(t,Piont_Bezier);
                d_x = P_now.dX - P_last.dX;
                d_y = P_now.dY - P_last.dY;
                length += sqrt(d_x*d_x + d_y*d_y);
                P_last = P_now;
            }
            P_now = get_point_Bezier(1,Piont_Bezier);
            d_x = P_now.dX - P_last.dX;
            d_y = P_now.dY - P_last.dY;
            length += sqrt(d_x*d_x + d_y*d_y);
        
        /*
        //法二 simpson's 3/8 rule       //误差0.1，3，像素以内       
            double a_x = (Piont_Bezier[3].dX - Piont_Bezier[0].dX + 3*Piont_Bezier[1].dX - 3*Piont_Bezier[2].dX);
            double a_y = (Piont_Bezier[3].dY - Piont_Bezier[0].dY + 3*Piont_Bezier[1].dY - 3*Piont_Bezier[2].dY);
            double b_x = 3*(Piont_Bezier[0].dX - 2*Piont_Bezier[1].dX + Piont_Bezier[2].dX);
            double b_y = 3*(Piont_Bezier[0].dY - 2*Piont_Bezier[1].dY + Piont_Bezier[2].dY);
            double c_x = 3*(Piont_Bezier[1].dX-Piont_Bezier[0].dX);
            double c_y = 3*(Piont_Bezier[1].dY-Piont_Bezier[0].dY);
            double a = 9*(a_x*a_x + a_y*a_y);
            double b = 12*(a_x*b_x + a_y*b_y);
            double c = (4*(b_x*b_x + b_y*b_y) + 6*(a_x*c_x + a_y*c_y));
            double d = 4*(b_x*c_x + b_y*c_y);
            double e = (c_x*c_x + c_y*c_y);

            double f_0 = sqrt(e);
            double f_1 = sqrt(a+b+c+d+e);
            double f_1_3 = sqrt(a/81 + b/27 + c/9 + d/3 + e);
            double f_2_3 = sqrt(16*a/81 + 8*b/27 + 4*c/9 + 2*d/3 + e);

            length = (f_0 + 3*f_1_3 + 3*f_2_3 +f_1)/8;
        */
            break;
        }
        case EPathType::PathType_Line:                                                        //直线
        {
            //两站点间直线距离
            length = station_line_length(t_path.iStartStation,t_path.iEndStation);
            break;
        }
        case EPathType::PathType_Circle:                                                        //圆
            break;
    }
    return length;
}

double CAGVMapCtrl::station_line_length(int station1_ID, int station2_ID)
{
    double length;
    double delta_x;
    double delta_y;
    //计算直线距离
    auto it_start = m_tMapInfo.mapStation.find(station1_ID);  
    auto it_end = m_tMapInfo.mapStation.find(station2_ID);   
    delta_x = (*it_start).second.tPt.dX - (*it_end).second.tPt.dX;
    delta_y = (*it_start).second.tPt.dY - (*it_end).second.tPt.dY;
    length = sqrt(delta_x*delta_x + delta_y*delta_y);                        
    return length;
}

void CAGVMapCtrl::update_openlist(int ID_station,int iDesStation)
{
    std::map<int, std::list<TPath>>::iterator Pit; //获取路径信息的迭代器
    bool join_openlist = false;                    //判断站点是否加入Openlist

    Pit = m_tMapInfo.mapadjMap.find(ID_station);                                            //指向站点对应的<int,list<TPath>>
    for (auto it_list = (*Pit).second.cbegin(); it_list != (*Pit).second.cend(); ++it_list) //it_list指向每条Tpath
    {
        if ((((*Pit).first == (*it_list).iStartStation) && ((*it_list).eDirect != EPathDirect::PathDirect_EndToStart)) || (((*Pit).first == (*it_list).iEndStation) && ((*it_list).eDirect != EPathDirect::PathDirect_StartToEnd))) //索引站点路线方向可行
        {
            if ((*Pit).first == (*it_list).iStartStation)                 
            {
                m_tAStartStation.iStationID = (*it_list).iEndStation;
            }
            else
            {
                m_tAStartStation.iStationID = (*it_list).iStartStation;
            }
            auto fit = Closelist.find(m_tAStartStation.iStationID);
            if ( fit == Closelist.end())                       //不在Closelist中
            {
                
                m_tAStartStation.Lengh_TPathID = path_length(*it_list);
                m_tAStartStation.s_g = Openlist[ID_station].s_g + m_tAStartStation.Lengh_TPathID;
                m_tAStartStation.s_f = station_line_length(m_tAStartStation.iStationID, iDesStation);
                m_tAStartStation.s_f += m_tAStartStation.s_g;

                fit = Openlist.find(m_tAStartStation.iStationID);
                if (fit == Openlist.end())                    //不在Openlist中
                {                  
                    m_tAStartStation.fStationID = ID_station;
                    m_tAStartStation.TPathID = (*it_list).iPathID;
                    
                    Openlist[m_tAStartStation.iStationID] = m_tAStartStation;
                }
                else     //在Openlist中
                {
                   if (m_tAStartStation.s_f < (*fit).second.s_f)                //经选定站点到达该站点具有更小的F值,更新父站点
                   {
                        (*fit).second.Lengh_TPathID = m_tAStartStation.Lengh_TPathID;
                        (*fit).second.s_g = m_tAStartStation.s_g;
                        (*fit).second.s_f = m_tAStartStation.s_f;
                        (*fit).second.fStationID = ID_station;
                        (*fit).second.TPathID = (*it_list).iPathID;
                   }
                }
            }          
        }
    }
}

int CAGVMapCtrl::find_select_station()
{
    static int min_f_ID;    //最小f值的站点ID
    static int init_f;     //对应f值

    auto it_ostart = Openlist.begin();
    init_f = it_ostart->second.s_f;                           //初值
    min_f_ID = it_ostart->second.iStationID;    //初值
    for (auto& it_openlist: Openlist)
    {
        if (it_openlist.second.s_f < init_f)
        {
            init_f = it_openlist.second.s_f ;
            min_f_ID = it_openlist.second.iStationID;
        }
        else if(fabs(it_openlist.second.s_f- init_f)<= EPS_0)      //认为相等
        {
            it_ostart = Openlist.find(min_f_ID);
            if (it_openlist.second.s_g >= ((*it_ostart).second.s_g))    //不相等时选离目标直线距离最短的,相等选最后加入openlist的
            {
                init_f = it_openlist.second.s_f ;
                min_f_ID = it_openlist.second.iStationID;
            } 
        }
    }
    return min_f_ID;
}

std::vector<TPointd> CAGVMapCtrl::line_path_interpolation(TPointd start,TPointd end)
{
    std::vector<TPointd> line_Path;
    TPointd in_point;     
    int div_n = (int)(sqrt((end.dY-start.dY)*(end.dY-start.dY) +(end.dX-start.dX)*(end.dX-start.dX) )/0.7);              //每0.7像素插值一个点分的份数
    double delta_x;
    double delta_y;

    if (div_n == 0)        //不用再插值
    {
        line_Path.push_back(end);             //起点不用放入
    }
    else
    {
        delta_x = (end.dX-start.dX)/div_n;
        delta_y = (end.dY-start.dY)/div_n;
        for (int i = 1; i <div_n; i++)                 //起点为已到达位置,不存入路径规划中
        {
            in_point.dX = start.dX + i*delta_x;
            in_point.dY = start.dY + i*delta_y;
            line_Path.push_back(in_point);                //在末尾插入点
        }
        line_Path.push_back(end);                       //将终点加入路径点
    }
    
    return line_Path;
}

std::vector<TPointd> CAGVMapCtrl::ID_path_interpolation(TPath t_path,int end_stationID,double Lengh_TPathID)
{
    std::vector<TPointd> Path; 
    std::map<int, TStation>::const_iterator it; //获取站点信息
    TPointd start;
    //Bezier相关参数
    TPointd Piont_Bezier[4];                 //存放4个控制点
    double delta_s = 0.7;                         //预分曲线的段长,实际不保证,只能接近
    double delta_t = delta_s/Lengh_TPathID;               //预分控制参数增量
    double pre_t;        //前一个点的控制参数
    double t;               //控制参数t
    TPointd pre_p;             //前一个点
    double threshold  = 0.1;             //允许实际分段的长度波动
    bool end_flg = false;                   //插值到终点的标志
    double int_h = 0.0;                //两插值点间的直线距离

    //规划时需考虑路径描述的起始点与选择规划的路径起始点不是一回事,
    if (t_path.eType == EPathType::PathType_Line)                           //直线插值
    {   
        it = m_tMapInfo.mapStation.find(t_path.iStartStation);
        start = (*it).second.tPt;
        it = m_tMapInfo.mapStation.find(t_path.iEndStation);
        if (t_path.iStartStation == end_stationID)                 //实际规划从t_path的终点到起点
        {
            Path = line_path_interpolation((*it).second.tPt,start);
        }
        else
        {
            Path = line_path_interpolation(start,(*it).second.tPt);
        }   
    }
    else if(  t_path.eType == EPathType::PathType_Bezier)    //贝塞尔曲线插值
    {
        //实际插值放入start
       
        //获取四个控制点
        it = m_tMapInfo.mapStation.find(t_path.iStartStation);  
        Piont_Bezier[0] = it->second.tPt;
        Piont_Bezier[1] = t_path.udata.tBazierData.tPtNearStart;
        Piont_Bezier[2] = t_path.udata.tBazierData.tPtNearEnd;
        it = m_tMapInfo.mapStation.find(t_path.iEndStation);   
        Piont_Bezier[3] = it->second.tPt;
        //初始化参数
        pre_t = 0.0;
        pre_p = Piont_Bezier[0];
        Path.push_back(pre_p);
        auto it_start = Path.begin();             
        while (!end_flg)
        {
            t = fmin(pre_t + delta_t,1.0);
            if(fabs(1.0- t)<= EPS_0)      //认为t == 1
            {
                start = Piont_Bezier[3];
                end_flg = true;
            }
            else
            {
                start = get_point_Bezier(t,Piont_Bezier);
            }
            int_h = sqrt((start.dY-pre_p.dY)*(start.dY-pre_p.dY) +(start.dX-pre_p.dX)*(start.dX-pre_p.dX));
            if ((int_h <= (delta_s - threshold)) || (int_h > (delta_s + threshold/2.0)))             //长度不合适,重新计算0.6-0.75不用重新计算
            {
                if (end_flg)          
                {
                    t = pre_t + (1 - pre_t) *delta_s/int_h;
                    end_flg = false;
                }
                else
                {
                    t = pre_t + delta_t *delta_s/int_h;
                }
                t = fmin(t,1.0);
                if(fabs(1.0- t)<= EPS_0)      //重新计算依然==1,说明到达终点了
                {
                    end_flg = true;
                    start = Piont_Bezier[3];
                    if (int_h <= threshold)            //终点到前一个点距离很小时，舍弃前一个点，直接设为终点
                    {
                        if (t_path.iStartStation == end_stationID)
                        {
                            Path.erase (Path.begin());          //头部删除
                        }
                        else
                        {              
                            Path.erase (Path.end()-1);          //在末尾删除点
                        }
                    } 
                }
                else
                {
                    start = get_point_Bezier(t,Piont_Bezier);
                } 
            }
            //将计算插值点放入路径
            if (t_path.iStartStation == end_stationID)
            {
                it_start = Path.begin();                          //头部插入
                it_start = Path.insert ( it_start , start);
            }
            else
            {
                Path.push_back(start);                //在末尾插入点
            }
            //更新值
            pre_t = t;
            pre_p = start;
        }
        
        //始点终点都在path中,插值完毕后,删除始点即第一个点
        Path.erase (Path.begin());
    }
    else                  //圆
    {
        /* code */
    }
    
    return Path;
}

void CAGVMapCtrl::MapToWorld(double dxMap, double dyMap, double &dx, double &dy)
{
    double deltaX, deltaY;
    if (dxMap > 0){
        deltaX = 0.5;
    }else{
        deltaX = -0.5; //负值应该减去！！！
    }
    if (dyMap > 0){
        deltaY = 0.5;
    }else{
        deltaY = -0.5;
    }
    dx = m_tMapInfo.dOriginXOffset + ((m_tMapInfo.iWidth)/2.0 +dxMap+deltaX)*m_tMapInfo.dResolution;
    dy = m_tMapInfo.dOriginYOffset + ((m_tMapInfo.iHeight)/2.0 +dyMap+deltaY)*m_tMapInfo.dResolution;
}

bool CAGVMapCtrl::WorldToMap(double dx, double dy, double &dxMap, double &dyMap)
{
    if (dx<m_tMapInfo.dOriginXOffset || dy<m_tMapInfo.dOriginYOffset)   //出界
    {
        return false;
    }
    dxMap = ((dx - m_tMapInfo.dOriginXOffset)/m_tMapInfo.dResolution - (m_tMapInfo.iWidth)/2.0);
    dyMap = ((dy - m_tMapInfo.dOriginYOffset)/m_tMapInfo.dResolution - (m_tMapInfo.iHeight)/2.0);
    if ((dxMap < m_tMapInfo.iWidth/2.0) && (dxMap > -m_tMapInfo.iWidth/2.0) && (dyMap < m_tMapInfo.iHeight/2.0) && (dyMap > -m_tMapInfo.iHeight/2.0))           //在范围内
    {
        return true;
    }
    else
    {
        return false;
    }  
}

std::vector<TPointd> CAGVMapCtrl::get_path(TPointd tAGVPos, int iDesStation, double &dRotateToAngle)
{
    std::vector<TPointd> vecPath; //存储总的路径点
    std::vector<TPointd> IDPath;     //存储一段路径的路径点
    int StartStationID = 0; //agv在站点上时,起始站点ID
    int SelectStationID = 0;                //选定站点ID
    bool atStation = false; //是否在站点上
    double delta_x;
    double delta_y;
    TPointd cur_tAGVPos;    //像素坐标系下的小车当前位置坐标
    std::map<int, TStation>::const_iterator it; //获取站点信息
    const TPath *p_tStartPath = nullptr; //第一段要走的路径

    Openlist.clear();
    Closelist.clear();
    
    //临时打印输出使用
    //std::ofstream out("piont.txt",std::ios::app);           //从文件末尾开始写
    //
    //tAGVPos 转到中心点在0,0的像素坐标系中
    WorldToMap(tAGVPos.dX,tAGVPos.dY, cur_tAGVPos.dX,cur_tAGVPos.dY);
    //判断tAGVPos是否在站点上
    for (it = (m_tMapInfo.mapStation.cbegin()); it != m_tMapInfo.mapStation.cend(); ++it)
    {
        delta_x = it->second.tPt.dX - cur_tAGVPos.dX;
        delta_y = it->second.tPt.dY - cur_tAGVPos.dY;
        if ((delta_x * delta_x + delta_y * delta_y) < TOLERANCE_PT_ON_STATION*TOLERANCE_PT_ON_STATION) //在范围内
        {
            StartStationID = it->first;
            atStation = true;
            break;
        }
    }
  
    if (atStation)                                //1.在站点上
    {
        if (StartStationID != iDesStation) //起始站点!=目标站点 , 使用A*寻找路径ID
        {
            m_tStartStation = m_tMapInfo.mapStation.find(iDesStation)->second; 
            m_tEndStation = m_tMapInfo.mapStation.find(StartStationID)->second; 
            
            m_tAStartStation.iStationID = StartStationID;
            m_tAStartStation.s_g = 0.0;
            Openlist[StartStationID] = m_tAStartStation; //将起始站点放入openlist
            update_openlist(StartStationID,iDesStation);
            Closelist[StartStationID] = Openlist[StartStationID]; //起始站点移入Closelist
            Openlist.erase(StartStationID);                       //Openlist删除起始站点
            while (true)                                  //找到最终结果才会跳出循环      没有可达路径或者找到路
            {
                if (Openlist.empty())
                {
                    vecPath.clear();//没有路径可达
                    break;
                }
                else 
                {
                    SelectStationID = find_select_station();                       //openlist中找到F值最小的站点
                    
                    if (SelectStationID == iDesStation)                   //选定站点是否为目标站点
                    {
                        //Closelist中回溯构建路径     
                        Closelist[SelectStationID] = Openlist[SelectStationID]; //选定站点移入Closelist
                        auto it_find_path = Closelist.find(SelectStationID);
                        auto it_insert = vecPath.begin();
                        auto it_list_path = m_tMapInfo.mapadjMap.find(it_find_path->second.fStationID);                  //路径list索引<int,list<TPath>>
                        auto it_list_one = (*it_list_path).second.cbegin();               //it_list_one指向一条Tpath的迭代器
                        while (SelectStationID != StartStationID)
                        {
                            for (it_list_one = (*it_list_path).second.cbegin(); it_list_one != (*it_list_path).second.cend(); ++it_list_one) //查找路径信息 it_list_one指向每条Tpath
                            {
                                if ((*it_list_one).iPathID == (*it_find_path).second.TPathID)              //找到对应路径信息
                                {
                                    IDPath = ID_path_interpolation(*it_list_one,SelectStationID, it_find_path->second.Lengh_TPathID);
                                    p_tStartPath = &(*it_list_one); 
                                    break;
                                }
                            }
                            it_insert = vecPath.begin();
                            vecPath.insert (it_insert,IDPath.begin(),IDPath.end());              //在总路径前面插入当前路段插值点
                            IDPath.clear();
                            SelectStationID =  it_find_path->second.fStationID;
                            it_find_path = Closelist.find(SelectStationID);
                            it_list_path = m_tMapInfo.mapadjMap.find(it_find_path->second.fStationID);    
                        }
                        break;
                    }
                    else
                    {
                        update_openlist(SelectStationID,iDesStation);                  
                        Closelist[SelectStationID] = Openlist[SelectStationID]; //选定站点移入Closelist
                        Openlist.erase(SelectStationID);                       //Openlist删除选定站点
                    }
                }
            }
        }
        else //小车在目标站点上,不用规划路径
        {
            vecPath.clear(); //就在目标位置
        }
    }
    else //2.不在站点上
    {
        //agv走直线到目标点
        it = m_tMapInfo.mapStation.find(iDesStation); //指向目标站点
        vecPath = line_path_interpolation(cur_tAGVPos, (*it).second.tPt);
    }

 //将数据保存到point.txt
    // for (int i = 1; i < 14; i++)
    // {
    //     out<< m_tMapInfo.mapStation[i].tPt.dX <<","<<m_tMapInfo.mapStation[i].tPt.dY << std::endl;
    // }
    // for (int i = 0; i < vecPath.size(); i++)
    // {
    //     out<< vecPath[i].dX <<","<<vecPath[i].dY << std::endl;
    // }
    //转成世界坐标系  
    for (int i = 0; i < vecPath.size(); i++)
    {
        MapToWorld(vecPath[i].dX,vecPath[i].dY,vecPath[i].dX,vecPath[i].dY);  
    }

    //计算初始旋转角度(用像素坐标系)
    if (vecPath.size() != 0)
    {
        auto itDesStation = m_tMapInfo.mapStation.find(iDesStation);
        if (p_tStartPath == nullptr) //没有在站点上
        {   
            dRotateToAngle = TwoPointAngle(cur_tAGVPos.dX, cur_tAGVPos.dY, itDesStation->second.tPt.dX, itDesStation->second.tPt.dY);
        }
        else
        {
            if (p_tStartPath->eType == EPathType::PathType_Line)
            {
                dRotateToAngle = TwoPointAngle(cur_tAGVPos.dX, cur_tAGVPos.dY, itDesStation->second.tPt.dX, itDesStation->second.tPt.dY);
            }
            else if (p_tStartPath->eType == EPathType::PathType_Bezier)
            {
                if (iDesStation == p_tStartPath->iStartStation)
                {
                    dRotateToAngle = TwoPointAngle(cur_tAGVPos.dX, cur_tAGVPos.dY, p_tStartPath->udata.tBazierData.tPtNearEnd.dX, p_tStartPath->udata.tBazierData.tPtNearEnd.dY);
                }
                else
                {
                    dRotateToAngle = TwoPointAngle(cur_tAGVPos.dX, cur_tAGVPos.dY, p_tStartPath->udata.tBazierData.tPtNearStart.dX, p_tStartPath->udata.tBazierData.tPtNearStart.dY);
                }
            }
        }
    }

    return vecPath; //编译器会进行RVO优化，不会调用拷贝构造函数了。
}

double CAGVMapCtrl::TwoPointAngle(double dx1, double dy1, double dx2, double dy2)
{
    double deltaX = dx2 - dx1;
    double deltaY = dy2 - dy1;
    double dis = std::sqrt(deltaX*deltaX + deltaY*deltaY);
    double dret = 0.0;

    if (dis>-0.0001 && dis<0.0001)
    {
        return 0.0;
    }

    dret = std::acos(deltaX / dis); //acos返回0~PI
    //dret = RadToDeg(dret);

    if (deltaY < 0)
    {
        dret = -dret;
    }

    return dret;
}


