#ifndef CMAPMANAGER_H
#define CMAPMANAGER_H
#include "AGVClientCore_global.h"
#include "AGVClient_def.h"
#include "SingletonMacro_Def.h"
#include "agv_msg_def.h"

enum class EMapManagerErr{
    Sucess = 0,
    FileNotExist, //文件不存在
    JsonError,    //json格式错误
    CreateFileFailed, //创建文件失败
};

class AGVCLIENTCORE_EXPORT CMapManager
{
    SINGLETON_DECLARE(CMapManager)
public:
    CMapManager();

    //返回当前地图信息
    TMapInfo &GetCurrentMapInfo(){return m_tCurrentMap;};
    //设置当前地图
    void SetCurrentMapInfo(TMapInfo tInfo);
    //返回扫描地图
    TMapInfo GetScanMapInfo(){return m_tScanMap;};

    //返回Current地图某个站点
    TStation *GetStation(int iStationID);

    //返回Current地图某条路线
    TPath *GetPath(int iPathID);

    //更新扫描地图数据
    void UpdateScanMap(TReportGripMap *pMap);

    //停止扫图，更新地图数据
    void UpdateScanStopMap(TReportGripAndStreamMap *pMap);

    //实时建图覆盖当前地图
    void ScanMapCoverCurrentMap();

    //检查地图

    //添加路线
    TPath *AddPath(EPathType eType, int iStartStation, int iEndStation);

    /**
     * @brief AddPathExisting : 添加已有的路线
     */
    void AddPathExisting(TPath tPath);

    //删除路线
    void DeletePath(int iPathID);

    /**
     * @brief AddStation : 添加站点
     * @param eType : 站点类型
     * @param tPt : 站点的map坐标（scene坐标）
     * @param dAngle : 站点的朝向角，单位°
     * @return
     */
    TStation *AddStation(EStationType eType, TPointd tPt, double dAngle);

    /**
     * @brief AddStationExisting : 添加已存在的站点
     * @param tStation
     */
    void AddStationExisting(TStation tStation);

    /**
     * @brief DeleteStation : 删除站点
     * @param pStation
     * @return 返回连接的路线
     */
    void DeleteStation(int iStationID);

    /**
     * @brief GetStationLinkedPath : 获取站点连接的路线
     * @return
     */
    std::list<TPath *> GetStationLinkedPath(int iStationID);

    //擦除部分数据

    /**
     * @brief ReadMapFromJson : 从当前AGV目录中的json文件中读取地图相关数据，数据读取到了AGV中的map中
     * @param strAGVID
     * @return AGVManager_Err_FileNotExist,AGVManager_Err_JsonError
     */
    int ReadMapJsonFromAGVDir(TAGVInfo *pInfo);

    /**
     * @brief ReadMapJsonFromSpecifyFile : 从指定的文件中读取地图数据，数据读取到m_currentmap中
     * @param strFileName
     * @return AGVManager_Err_FileNotExist,AGVManager_Err_JsonError
     */
    int ReadMapJsonFromSpecifyFile(QString strFileName);
    int ReadMapJsonFromSpecifyFile(QByteArray &buffer);

    /**
     * @brief SaveMap: 将指定的地图保存到本地，并修改agv基础信息文件
     * @param strFilePath：地图路径
     * @return
     */
    int SaveMap(QString strFilePath, QString strMapName);
    /**
     * @brief SaveMap:将currentmap保存到agvinfo中的json中，文件名是agvinfo中的
     * @return
     */
    int SaveMap();

    /**
     * @brief InitDefaultMap：没有地图文件，初始默认小地图
     */
    void InitDefaultMap();


    //像素坐标转世界坐标,这里假定像素坐标(0,0)位于地图中心，和scene正好重合(注意不是View坐标系)
    void MapToWorld(double dxMap, double dyMap, double &dx, double &dy);
    //世界坐标转像素坐标
    bool WorldToMap(double dx, double dy, double &dxMap, double &dyMap);

    //地图data索引转pixmap行列,这里假设data数据从地图左上角开始，这里pixmap原点也在左上角
    void DataIndexToPixmapRowCol(int index, int iWidth, int iHeight, int &iRow, int &iCol);
    //pixmap行列转地图data索引
    void PixmapRowColToDataIndex(int iRow, int iCol, int iWidth, int iHeight, int &index);

    //Pixmap行列转scene
    void ColRowPixmapToScene(int iPixRow, int iPixCol, int &iSceneRow, int &iSceneCol);


    //地图data索引转视图行列,这里假设data数据从地图左上角开始，这里Item原点在中心，需考虑平移
    void DataIndexToItemRowCol(int index, int iWidth, int iHeight, int &iRow, int &iCol);
    //视图行列转地图data索引
    void ItemRowColToDataIndex(int iRow, int iWidth, int iHeight, int iCol, int &index);

private:
    //读取指定路径的地图
    int ReadMapData(QString strMapPath, QByteArray &data);

private:

    //当前地图信息
    TMapInfo m_tCurrentMap;

    //扫描的地图信息
    TMapInfo m_tScanMap;

    //pbstream数据 cartographer 需要
    unsigned char *m_ppbstreamScan;
    int m_iLenpbstreamScan;
    unsigned char *m_ppbstream;
    int m_iLenpbstream;



};

#endif // CMAPMANAGER_H
