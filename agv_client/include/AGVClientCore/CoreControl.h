#ifndef CCORECONTROL_H
#define CCORECONTROL_H
#include <string>
#include "SingletonMacro_Def.h"
#include "AGVClientCore_global.h"

//AGV运行状态
enum class EAGVRunStatus{
    DoNothing,    //什么都不干
    Scan_Start,   //扫图开始
    Scan_RecvMap, //扫图收到一帧数据
    Run_GoToDes,  //导航去目标点
};

class AGVCLIENTCORE_EXPORT CCoreControl
{
    SINGLETON_DECLARE(CCoreControl)
public:
    CCoreControl();

    //返回当前AGV运行状态
    EAGVRunStatus GetAGVRunStatus(){return m_agvStatus;};
    void SetAGVRunStatus(EAGVRunStatus status){m_agvStatus = status;}; //设置状态

    //开始扫图
    void StartScanMap(bool bScanMap);
    bool IsScanMap();

    //收到一帧数据，更新状态
    void UpdateStatusByRecvScanMap();

    //设置/获取当前正在编辑地图的AGVID
    void SetCurrentMapAGVID(std::string strAGVID){m_strAGVID=strAGVID;};
    std::string GetCurrentMapAGVID(){return m_strAGVID;};

    //当前地图被修改标记
    void SetCurrentMapModify(bool bModify){m_bCurrentMapModify=bModify;};
    bool GetCurrentMapModify(){return m_bCurrentMapModify;};

    //当前是否在编辑地图
    void SetEditMap(bool bEdit){m_bIsEditMap = bEdit;};
    bool GetEditMap(){return m_bIsEditMap;};

private:
    //AGV运行状态
    EAGVRunStatus m_agvStatus;
    //当前正在编辑的AGVID
    std::string m_strAGVID;
    //当前地图是否被修改
    bool m_bCurrentMapModify;
    //当前是否正在编辑地图
    bool m_bIsEditMap;
};

#endif // CCORECONTROL_H
