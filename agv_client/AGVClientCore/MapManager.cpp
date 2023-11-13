#include <QDebug>
#include <QFile>
#include <QDir>
#include <QFileInfo>
#include <QDateTime>
#include "MapManager.h"
#include "GlogWrapper.h"
#include "CoreControl.h"
#include "GoerToolUtile.h"
#include "json.hpp"
#include "AGVManager.h"
#include "b64.h"

using json = nlohmann::json;

SINGLETON_IMPLEMENT(CMapManager)

CMapManager::CMapManager()
{
//    //构造测试数据
//    m_tCurrentMap.iWidth = 400;
//    m_tCurrentMap.iHight = 100;
//    for (int i=0; i<m_tCurrentMap.iWidth*m_tCurrentMap.iHight; i++){
//        if (i % 2 == 0)
//        {
//            int ix, iy;
//            DataIndexToPixmapRowCol(i, m_tCurrentMap.iWidth, m_tCurrentMap.iHight, ix, iy);
//            m_tCurrentMap.vecUnknown.push_back(TPoint{ix, iy});
//            continue;
//        }
//        if (i % 3 == 0)
//        {
//            int ix, iy;
//            DataIndexToPixmapRowCol(i, m_tCurrentMap.iWidth, m_tCurrentMap.iHight, ix, iy);
//            m_tCurrentMap.vecUnknown.push_back(TPoint{ix, iy});
//            continue;
//        }
//    }
//    m_tCurrentMap.dResolution = 0.05;
//    m_tCurrentMap.dOriginXOffset = -5;
//    m_tCurrentMap.dOriginYOffset = -5;

    m_ppbstream = nullptr;
    m_ppbstreamScan = nullptr;

    m_iLenpbstream = 0;
    m_iLenpbstreamScan = 0;
}

void CMapManager::SetCurrentMapInfo(TMapInfo tInfo)
{
    m_tCurrentMap=tInfo;
}

TStation *CMapManager::GetStation(int iStationID)
{
    auto itStation = m_tCurrentMap.mapStation.find(iStationID);
    if (itStation != m_tCurrentMap.mapStation.end())
    {
        return &itStation->second;
    }
    else
    {
        return nullptr;
    }
}

TPath *CMapManager::GetPath(int iPathID)
{
    auto itPath = m_tCurrentMap.mapPath.find(iPathID);
    if (itPath != m_tCurrentMap.mapPath.end())
    {
        return &itPath->second;
    }
    else
    {
        return nullptr;
    }
}

void CMapManager::UpdateScanMap(TReportGripMap *pMap)
{
    m_tScanMap.dResolution = pMap->fResolution; //分辨率 m
    m_tScanMap.iHight = pMap->iHight; //高度(像素)
    m_tScanMap.iWidth = pMap->iWidth;  //宽度(像素)
    m_tScanMap.dOriginXOffset = pMap->dOriginXOffset; //地图左下角x偏移，由此可计算出地图(0,0)点位置，单位m
    m_tScanMap.dOriginYOffset = pMap->dOriginYOffset; //地图左下角y偏移
    m_tScanMap.vecOccupy.clear();
    m_tScanMap.vecUnknown.clear();

    //更新两个缓冲区
    unsigned char *pMapdata = ((unsigned char*)pMap)+sizeof (TReportGripMap);
    for (int i=0; i<pMap->iWidth*pMap->iHight; i++)
    {
        int ix, iy;
        if (pMapdata[i] > 60 && pMapdata[i]<255)
        {
            DataIndexToPixmapRowCol(i, pMap->iWidth, pMap->iHight, ix, iy);
            m_tScanMap.vecOccupy.push_back(TPoint{ix, iy});
        }
        else if (pMapdata[i] == 255)
        {
            DataIndexToPixmapRowCol(i, pMap->iWidth, pMap->iHight, ix, iy);
            m_tScanMap.vecUnknown.push_back(TPoint{ix, iy});
        }
        else{}
    }
}

void CMapManager::UpdateScanStopMap(TReportGripAndStreamMap *pMap)
{
    TReportGripMap *pGridMap = &pMap->tmap;
    //更新网格地图数据
    UpdateScanMap(pGridMap);

    //记录pbstream数据
    if (m_ppbstreamScan != nullptr)
    {
        free(m_ppbstreamScan);
        m_ppbstreamScan = nullptr;
        m_iLenpbstreamScan = 0;
    }

    //拷贝pbstream数据
    unsigned char *pPbstreamHeader = ((unsigned char *)pMap + sizeof (TReportGripMap) + pGridMap->iHight*pGridMap->iWidth);
    m_iLenpbstreamScan = *((int*)(pPbstreamHeader));
    m_ppbstreamScan = (unsigned char*)malloc(m_iLenpbstreamScan);
    memcpy(m_ppbstreamScan, pPbstreamHeader+4, m_iLenpbstreamScan);

}

void CMapManager::ScanMapCoverCurrentMap()
{
    //替换当前扫描地图到编辑的地图
    SetCurrentMapInfo(m_tScanMap);

    //替换pbstream缓存
    if (m_ppbstream != nullptr)
    {
        free(m_ppbstream);
        m_ppbstream = nullptr;
    }
    m_ppbstream = (unsigned char *)malloc(m_iLenpbstreamScan);
    memcpy(m_ppbstream, m_ppbstreamScan, m_iLenpbstreamScan);
    m_iLenpbstream = m_iLenpbstreamScan;
}

TPath *CMapManager::AddPath(EPathType eType, int iStartStation, int iEndStation)
{
    if (iStartStation == iEndStation)
    {
        assert(false);
        return nullptr;
    }
    auto itStartStation = m_tCurrentMap.mapStation.find(iStartStation);
    auto itEndStation = m_tCurrentMap.mapStation.find(iEndStation);
    TPointd tStart = itStartStation->second.tPt;
    TPointd tEnd = itEndStation->second.tPt;

    TPath tpath;
    tpath.eType = eType;
    tpath.iStartStation = itStartStation->second.iStationID;
    tpath.iEndStation = itEndStation->second.iStationID;
    tpath.eDirect = EPathDirect::PathDirect_Bothway;
    if (!m_tCurrentMap.mapPath.empty())
    {
        auto itEnd = m_tCurrentMap.mapPath.end();
        itEnd--;
        tpath.iPathID = itEnd->first + 1;
    }
    else
    {
        tpath.iPathID = 0;
    }
    if (eType == EPathType::PathType_Circle){
        //计算圆心数据
        //tpath.udata.tArcData.tPtCenter.iX = ;
    }else if (eType == EPathType::PathType_Bezier){

        tpath.udata.tBazierData.tPtNearStart.dX = (itEndStation->second.tPt.dX + 2*itStartStation->second.tPt.dX)/3;
        tpath.udata.tBazierData.tPtNearStart.dY = (itEndStation->second.tPt.dY + 2*itStartStation->second.tPt.dY)/3;
        tpath.udata.tBazierData.tPtNearEnd.dX = (2*itEndStation->second.tPt.dX + itStartStation->second.tPt.dX)/3;
        tpath.udata.tBazierData.tPtNearEnd.dY = (2*itEndStation->second.tPt.dY + itStartStation->second.tPt.dY)/3;
    }

    auto ret = m_tCurrentMap.mapPath.insert(make_pair(tpath.iPathID, tpath));
    if (!ret.second){
        return nullptr;
    }
    return &ret.first->second;

}

void CMapManager::AddPathExisting(TPath tPath)
{
    m_tCurrentMap.mapPath.insert(make_pair(tPath.iPathID, tPath));
}

void CMapManager::DeletePath(int iPathID)
{
    auto it = m_tCurrentMap.mapPath.find(iPathID);
    if (it != m_tCurrentMap.mapPath.end())
    {
        m_tCurrentMap.mapPath.erase(it);
    }
}

TStation *CMapManager::AddStation(EStationType eType, TPointd tPt, double dAngle)
{
    TStation tstation;
    tstation.eType = eType;
    tstation.tPt = tPt;
    tstation.dAngle = dAngle;
    if (!m_tCurrentMap.mapStation.empty())
    {
        auto itStationEnd = m_tCurrentMap.mapStation.end();
        itStationEnd--;
        tstation.iStationID = itStationEnd->first + 1;
    }
    else
    {
        tstation.iStationID = 0;
    }

    auto ret = m_tCurrentMap.mapStation.insert(make_pair(tstation.iStationID, tstation));
    if (ret.second){
        return &ret.first->second;
    }else{
        return nullptr;
    }
}

void CMapManager::AddStationExisting(TStation tStation)
{
    m_tCurrentMap.mapStation.insert(std::make_pair(tStation.iStationID, tStation));
}

void CMapManager::DeleteStation(int iStationID)
{
    auto it = m_tCurrentMap.mapStation.find(iStationID);
    if (it != m_tCurrentMap.mapStation.end())
    {
        m_tCurrentMap.mapStation.erase(it);
    }
}

std::list<TPath *> CMapManager::GetStationLinkedPath(int iStationID)
{
    std::list<TPath *> listPath;
    auto itStation = m_tCurrentMap.mapStation.find(iStationID);
    if (itStation != m_tCurrentMap.mapStation.end())
    {
        for (auto &itPath : m_tCurrentMap.mapPath)
        {
            if (itPath.second.iEndStation == itStation->first
                    || itPath.second.iStartStation == itStation->first)
            {
                listPath.push_back(&itPath.second);
            }
        }
    }
    else
    {
        assert(false);
    }
    return listPath;
}

/*坐标系，这里Scene是像素坐标系，OriginX和OriginY是world坐标系：
 *    ^y
 *    |
 *    |--->x
 *
 *    -------------------------
 *    |              #(ix,iy) |
 *    |                       |
 *    |      #Scene(0,0)      |
 *    |                       |
 *    |                       |
 *    #------------------------
 *    m_tCurrentMap中OriginX和OriginY
 * 这里(ix,iy)对应像素坐标，那对应的dx求法：
 * 简单来说就是 OriginX+len，这个len是ix到OriginX像素长度转world长度！由图可知len=(width/2+ix)*精度，这里ix正负都满足
*/
void CMapManager::MapToWorld(double dxMap, double dyMap, double &dx, double &dy)
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

    TMapInfo *ptMapinfo = nullptr;
    EAGVRunStatus eStatus =  CCoreControl::GetInstance()->GetAGVRunStatus();
    if (eStatus != EAGVRunStatus::Scan_RecvMap)
    {
        ptMapinfo = &m_tCurrentMap;
    }
    else
    {
        ptMapinfo = &m_tScanMap;
    }
    dx = ptMapinfo->dOriginXOffset + ((ptMapinfo->iWidth)/2.0 +dxMap+deltaX)*ptMapinfo->dResolution;
    dy = ptMapinfo->dOriginYOffset + ((ptMapinfo->iHight)/2.0 +dyMap+deltaY)*ptMapinfo->dResolution;
}

bool CMapManager::WorldToMap(double dx, double dy, double &dxMap, double &dyMap)
{
    TMapInfo *ptMapinfo = nullptr;
    EAGVRunStatus eStatus =  CCoreControl::GetInstance()->GetAGVRunStatus();
    if (eStatus != EAGVRunStatus::Scan_RecvMap)
    {
        ptMapinfo = &m_tCurrentMap;
    }
    else
    {
        ptMapinfo = &m_tScanMap;
    }

    if (dx<ptMapinfo->dOriginXOffset || dy<ptMapinfo->dOriginYOffset){
        return false;
    }

    double dxTemp = (dx - ptMapinfo->dOriginXOffset)/ptMapinfo->dResolution - (ptMapinfo->iWidth)/2.0;
    double dyTemp = (dy - ptMapinfo->dOriginYOffset)/ptMapinfo->dResolution - (ptMapinfo->iHight)/2.0;

    dxMap = (dxTemp);
    dyMap = (dyTemp);

    if (dxMap<ptMapinfo->iWidth && dyMap<ptMapinfo->iHight){
//        qDebug() <<"dOriginXOffset:" << ptMapinfo->dOriginXOffset << "dOriginYOffset:" << ptMapinfo->dOriginYOffset
//                << "dResolution:" << ptMapinfo->dResolution << "iWidth:" << ptMapinfo->iWidth << "height:" << ptMapinfo->iHight;
        return true;
    }

    return false;
}

void CMapManager::DataIndexToPixmapRowCol(int index, int iWidth, int iHeight, int &iRow, int &iCol)
{
    //得到的数据坐标是(iColMap, iRowMap)，别弄反了
    iCol = index / iWidth;
    iRow = index % iWidth;
}

void CMapManager::PixmapRowColToDataIndex(int iRow, int iCol, int iWidth, int iHeight, int &index)
{
    index = iCol*iWidth + iRow;
}

void CMapManager::ColRowPixmapToScene(int iPixRow, int iPixCol, int &iSceneRow, int &iSceneCol)
{
    iSceneRow = iPixRow - m_tCurrentMap.iHight/2;
    iSceneCol = iPixCol - m_tCurrentMap.iWidth/2;
}

/*两个坐标系分别为coorMap、coorItem，单位都是像素：
*   ------------------▷x coorMap
*   |    dely
*   |delx ------------------▷x coorItem
*   |    |
*   |    |
*   ▽    |
*        |
*        ▽
* coorMap到coorItem的变换矩阵
*   1 0 delx
*A= 0 1 dely   由coorMap和coorItem相对位置，这里的delx为-Width/2，  dely为-Hight/2
*   0 0  1
*对于一个coorMap中的点(x,y)，可以通过矩阵相乘A*(x,y,1)计算出coorItem中点的位置
*/
void CMapManager::DataIndexToItemRowCol(int index, int iWidth, int iHeight, int &iRow, int &iCol)
{
    int iRowMap = index / iWidth;
    int iColMap = index % iWidth;
    //得到的数据坐标是(iColMap, iRowMap)，别弄反了

    iRow = iColMap - iWidth/2;
    iCol = iRowMap - iHeight/2;
}

//上面反推回去即可
void CMapManager::ItemRowColToDataIndex(int iRow, int iWidth, int iHeight, int iCol, int &index)
{
    int iRowMap = iCol + iWidth/2;
    int iColMap = iRow + iHeight/2;

    index = iRowMap*iWidth + iColMap;
}

int CMapManager::ReadMapData(QString strMapPath, QByteArray &data)
{
    QFileInfo fileinfo(strMapPath);
    if (!fileinfo.exists())
    {
        return (int)EMapManagerErr::FileNotExist;
    }
    QFile file(strMapPath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        return (int)EMapManagerErr::FileNotExist;
    }
    data = file.readAll();
    file.close();
    return 0;
}

int CMapManager::ReadMapJsonFromAGVDir(TAGVInfo *pInfo)
{
    QString strExeDir = CGoerToolUtile::GetExeDirectory();
    QString strAGVsDir = strExeDir + "/AGVs";
    QString strFile = strAGVsDir + "/" + pInfo->strAGVID.c_str() + "/maps/" + pInfo->strMapName.c_str() + ".json";
    QByteArray buff;
    int iret = 0;
    if (0 != (iret=ReadMapData(strFile, buff)))
    {
        return iret;
    }
    pInfo->strMapJson = buff.data();
    //计算MD5并校验 TODO

    return ReadMapJsonFromSpecifyFile(buff);
}

int CMapManager::ReadMapJsonFromSpecifyFile(QString strFileName)
{
    QByteArray buff;
    int iret = 0;
    if (0 != (iret=ReadMapData(strFileName, buff)))
    {
        return iret;
    }
    return ReadMapJsonFromSpecifyFile(buff);
}

int CMapManager::ReadMapJsonFromSpecifyFile(QByteArray &buffer)
{
    //解析json
    if (!json::accept(buffer.data()))
    {
        return (int)EMapManagerErr::JsonError;
    }
    auto itValue = json::parse(buffer.data());
    //解析地图头
    auto itmap_header = itValue.value("map_header", json::object());
    m_tCurrentMap.strMapName = itmap_header.value("map_name", "default");
    m_tCurrentMap.strVersion = itmap_header.value("ver", "1.0.0");
    m_tCurrentMap.dResolution = itmap_header.value("resolution", 0.05);
    m_tCurrentMap.iWidth = itmap_header.value("width", 100);
    m_tCurrentMap.iHight = itmap_header.value("height", 100);
    m_tCurrentMap.dOriginXOffset = itmap_header.value("originX", -5.0);
    m_tCurrentMap.dOriginYOffset = itmap_header.value("originY", -5.0);
    //解析地图数据
    m_tCurrentMap.vecUnknown.clear();
    m_tCurrentMap.vecOccupy.clear();
    auto itmap_data = itValue.value("map_data", "");
    int index = 0;
    for (auto &itV : itmap_data)
    {
        if (itV == MAP_EMPTY)
        {
            index++;
            continue;
        }
        int iRow, iCol;
        CMapManager::GetInstance()->DataIndexToPixmapRowCol(index++, m_tCurrentMap.iWidth, m_tCurrentMap.iHight, iRow, iCol);
        if (itV == MAP_UNKNOWN)
        {
            m_tCurrentMap.vecUnknown.push_back(TPoint{iRow, iCol});
        }
        else if (itV == MAP_OCCYPY)
        {
            m_tCurrentMap.vecOccupy.push_back(TPoint{iRow, iCol});
        }
        else{}
    }
    //解析站点
    m_tCurrentMap.mapStation.clear();
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

        m_tCurrentMap.mapStation.insert(make_pair(tStation.iStationID, tStation));
    }
    //解析路线
    m_tCurrentMap.mapPath.clear();
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

        m_tCurrentMap.mapPath.insert(make_pair(tPath.iPathID, tPath));
    }

    //解析pbstream数据
    std::string strPbstream = itValue.value("pbstream", "");
    if (m_ppbstream != nullptr)
    {
        free(m_ppbstream);
        m_iLenpbstream = 0;
    }
    m_ppbstream = b64_decode_ex(strPbstream.c_str(), strPbstream.length(), (size_t*)&m_iLenpbstream);

    return (int)EMapManagerErr::Sucess;
}

int CMapManager::SaveMap(QString strFilePath, QString strMapName)
{
    TMapInfo &tmap = m_tCurrentMap;
    tmap.strMapName = strMapName.toStdString();
    //创建json内容
    //地图头
    json jmap_header_value;
    jmap_header_value["map_name"] = strMapName.toStdString().c_str();
    jmap_header_value["ver"] = tmap.strVersion;
    jmap_header_value["resolution"] = tmap.dResolution;
    jmap_header_value["width"] = tmap.iWidth;
    jmap_header_value["height"] = tmap.iHight;
    jmap_header_value["originX"] = tmap.dOriginXOffset;
    jmap_header_value["originY"] = tmap.dOriginYOffset;
    //地图数据
    std::string strData;
    strData.resize(tmap.iWidth*tmap.iHight, MAP_EMPTY);
    for (auto &itPt : tmap.vecUnknown)
    {
        int index = 0;
        CMapManager::GetInstance()->PixmapRowColToDataIndex(itPt.iX, itPt.iY, tmap.iWidth, tmap.iHight,  index);
        strData[index] = MAP_UNKNOWN;
    }
    for (auto &itPt : tmap.vecOccupy)
    {
        int index = 0;
        CMapManager::GetInstance()->PixmapRowColToDataIndex(itPt.iX, itPt.iY, tmap.iWidth, tmap.iHight,  index);
        strData[index] = MAP_OCCYPY;
    }

    //站点数据
    json jstation_value;
    for (auto &itStation : tmap.mapStation)
    {
        json j;
        j["id"] = itStation.second.iStationID;
        j["type"] = itStation.second.eType;
        j["pos"]["x"] = itStation.second.tPt.dX;
        j["pos"]["y"] = itStation.second.tPt.dY;
        j["pos"]["angle"] = itStation.second.dAngle;
        j["comment"] = itStation.second.strComment;
        jstation_value.push_back(j);
    }
    //路线数据
    json jpath_value;
    for (auto &itPath : tmap.mapPath)
    {
        json j;
        j["id"] = itPath.second.iPathID;
        j["type"] = itPath.second.eType;
        j["start_station"] = itPath.second.iStartStation;
        j["end_station"] = itPath.second.iEndStation;
        j["direct"] = itPath.second.eDirect;
        if (itPath.second.eType == EPathType::PathType_Circle)
        {
            j["circle_center"]["x"] = itPath.second.udata.tArcData.tPtCenter.dX;
            j["circle_center"]["y"] = itPath.second.udata.tArcData.tPtCenter.dY;
            j["circle_center"]["isgood"] = itPath.second.udata.tArcData.bIsGood;
        }
        else if (itPath.second.eType == EPathType::PathType_Bezier)
        {
            j["start_control_pt"]["x"] = itPath.second.udata.tBazierData.tPtNearStart.dX;
            j["start_control_pt"]["y"] = itPath.second.udata.tBazierData.tPtNearStart.dY;
            j["end_control_pt"]["x"] = itPath.second.udata.tBazierData.tPtNearEnd.dX;
            j["end_control_pt"]["y"] = itPath.second.udata.tBazierData.tPtNearEnd.dY;
        }else{}
        jpath_value.push_back(j);
    }
    //pbstream数据
    char *pDataB64 = b64_encode(m_ppbstream, m_iLenpbstream);

    //整体json
    json jValue;
    jValue["map_header"] = jmap_header_value;
    jValue["map_data"] = strData;
    jValue["station_data"] = jstation_value;
    jValue["path_data"] = jpath_value;
    jValue["pbstream"] = pDataB64;
    std::string strJson = jValue.dump();

    //保存地图
    QFile file(strFilePath);
    if (!file.open(QIODevice::WriteOnly))
    {
        return (int)EMapManagerErr::FileNotExist;
    }
    file.write(strJson.c_str());
    file.flush();
    file.close();

    //修改AGV配套信息
    TAGVInfo *pAGV = CAGVManager::GetInstance()->GetAGV(CCoreControl::GetInstance()->GetCurrentMapAGVID());
    pAGV->strMapName = strMapName.toStdString();
    pAGV->strMapJson = strJson;
    //pAGV->strMapMD5 = ; TODO
    //保存一下
    return CAGVManager::GetInstance()->SaveAGVBaseInfo(pAGV);
}

int CMapManager::SaveMap()
{
    TAGVInfo *pAGVInfo = CAGVManager::GetInstance()->GetAGV(CCoreControl::GetInstance()->GetCurrentMapAGVID());
    QString strAGVMapsDir = CGoerToolUtile::GetExeDirectory() + "/AGVs/" + pAGVInfo->strAGVID.c_str() + "/maps/";
    QDir dir(strAGVMapsDir);
    if (!dir.exists())
    {
        if(!dir.mkpath(strAGVMapsDir))
        {
            return (int)EMapManagerErr::CreateFileFailed;
        }
    }
    QString strAGVMapPath = strAGVMapsDir + m_tCurrentMap.strMapName.c_str() + ".json";
    return SaveMap(strAGVMapPath, m_tCurrentMap.strMapName.c_str());
}

void CMapManager::InitDefaultMap()
{
    m_tCurrentMap.strMapName = QDateTime::currentDateTime().toString("yyyyMMddhhmmss").toStdString();
    m_tCurrentMap.iHight = 500;
    m_tCurrentMap.iWidth = 500;
    m_tCurrentMap.dResolution = 0.05;
    m_tCurrentMap.dOriginXOffset = 0;
    m_tCurrentMap.dOriginYOffset = 0;
    m_tCurrentMap.mapPath.clear();
    m_tCurrentMap.mapStation.clear();
    m_tCurrentMap.vecOccupy.clear();
    m_tCurrentMap.vecUnknown.clear();
    m_tCurrentMap.strVersion = "0.0.1";
}
