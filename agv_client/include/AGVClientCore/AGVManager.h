#ifndef CAGVMANAGER_H
#define CAGVMANAGER_H
#include <unordered_map>
#include "AGVClientCore_global.h"
#include "AGVClient_def.h"
#include "SingletonMacro_Def.h"
#include "agv_msg_def.h"

enum class EAGVManagerErr{
    Sucess = 0,
    FileNotExist, //文件不存在
    JsonError,    //json格式错误
    NoThisAGV,    //不存在此AGV
    CreateFileFailed, //创建文件失败
};

class AGVCLIENTCORE_EXPORT CAGVManager
{
    SINGLETON_DECLARE(CAGVManager)
public:
    CAGVManager();

    /**
     * @brief Init:初始化，读取本地AGV信息
     * @return
     */
    bool Init();

    /**
     * @brief GetAGV : 获取某个AGV的信息
     * @param strAGVID
     * @return
     */
    TAGVInfo *GetAGV(std::string strAGVID);
    TAGVInfo *GetCurrentAGV();

    ///
    /// \brief DeleteAGV
    /// \param strAGVID
    ///
    void DeleteAGV(std::string strAGVID);

    /**
     * @brief GetAllAGV 获取所有AGV信息
     * @return
     */
    const std::unordered_map<std::string, TAGVInfo> &GetAllAGV();

    /**
     * @brief AGVLogin 某个AGV的某个连接，连接了
     * @param pData
     */
    int AGVLogin(std::string strAGVID, std::string strIP, bool bSucess, TBroadcastResData *pData);

    /**
     * @brief UpdateAGVInfo
     * @param tData
     */
    void UpdateAGVInfo(std::string strAGVID, TAGVRealTimeData tData);

    /**
     * @brief AGVDisconnect AGV掉线
     * @param strAGVID
     */
    void AGVDisconnect(std::string strAGVID);

    /**
     * @brief SaveAGVBaseInfo : 保存agv基础数据到配置
     */
    int SaveAGVBaseInfo(TAGVInfo *pAGVInfo);

    /**
     * @brief SaveAGVConfigInfo : 保存AGV配置数据 TODO
     * @param pAGVInfo
     * @return
     */
    int SaveAGVConfigInfo(TAGVInfo *pAGVInfo);

private:
    //创建AGV本地配置文件
    int CreateAGVConfig(TAGVInfo *pAGVInfo);



private:
    std::unordered_map<std::string, TAGVInfo> m_mapAGVs;
};

#endif // CAGVMANAGER_H
