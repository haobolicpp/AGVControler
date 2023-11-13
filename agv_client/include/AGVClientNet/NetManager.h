#ifndef CNETMANAGER_H
#define CNETMANAGER_H
#include <thread>
#include <functional>
#include <unordered_map>
#include <list>
#include <mutex>
#include "AGVClientNet_global.h"
#include "SingletonMacro_Def.h"
#include "LockFreeRingBuffer.h"
#include "agv_msg_def.h"
#include "AutoIncBuffer.h"

//网络数据回应消息的回调，参数传递数据类型为TCallBackData*，必须new出来，UI层用完后delete，否则可能会在信号槽处理中丢失
typedef std::function<void(TCallBackData*)> ResponseCallFunc;
//某个AGV断开连接回调,参数:AGVID
typedef std::function<void(std::string)> DisconnectCallBackFunc;
//某个AGV连接接入回调，参数:AGVID,IP,是否连接成功,广播应答的额外数据
typedef std::function<void(std::string, std::string, bool, TBroadcastResData)> ConnectCallBackFunc;

class CAGVDiscovery;
class CTcpClientServer;
class AGVCLIENTNET_EXPORT CNetManager
{
    SINGLETON_DECLARE(CNetManager)
public:
    CNetManager();

    bool Init();

    //关闭网络通信线程等
    void Close();

/////控制指令
    //发送实时数据的应答
    bool SendRealTimeDataRes(std::string strAGVID);
    //开始建图指令
    bool SendScanMapReq(std::string strAGVID, bool bStartScanMap);
    //方向控制指令
    bool SendMoveCtrl(std::string strAGVID, int iData);
    //目标点控制
    bool SendGoToDes(std::string strAGVID, TAGVGoToDes tDes);
    //目标点取消
    bool SendCancelGoToDes(std::string strAGVID);
    //遥操作切换指令
    bool SendTeleoperation(std::string strAGVID, int iData);
/////文件操作指令
    //下发地图(json文件数据)，在收到回应之前,UI需保证不能下发新的文件
    bool SendMapData(std::string strAGVID, const char *pMapData, int ilen);
    //拉取地图
    bool SendUpLoadMapReq();

    //断线回调注册
    void RegisterDisconnect(DisconnectCallBackFunc func);
    //连接回调注册
    void RegisterConnect(ConnectCallBackFunc func);

    /**
     * @brief ListenMsg : not send data, only care response data.
     * @param iClass
     * @param iType
     * @param responseFunc
     */
    void ListenMsg(int iClass, int iType, ResponseCallFunc responseFunc);

    /**
     * @brief send broadcast data
     */
    void SendBroadcast();

private:
    //处理连接接收缓冲区，在通信线程中回调
    void OnIncRecvBuffer(TCallBackData *pData);
    void OnDecRecvBuffer(TCallBackData *pData);
    //AGV断开连接登记
    void AGVDisconnect(std::string strAGVID, ETCPClientServerType type);
    //AGV连接登记
    void AGVConnect(void *pConnInfo, ETCPClientServerType type, bool bSucess);
    //发送控制或文件异步数据
    bool SendAsyncDataWrap(TMsgHeader *pHeader, void *pData, int ilen);

private:
    //广播处理对象
    CAGVDiscovery *m_pAGVDiscovery;
    //控制指令TCP连接服务管理对象
    CTcpClientServer *m_pControlTcpClientServer;
    //文件指令TCP连接服务管理对象
    CTcpClientServer *m_pFileTcpClientServer;
    //断开连接回调对象
    DisconnectCallBackFunc m_funcDisconnect;
    //连接接入回调对象
    ConnectCallBackFunc m_funcConnect;
    //AGV连接状态管理,key:AGVID，value:两个连接的
    std::map<std::string, TAGVConnetcStatus> m_mapAGVConnectStatus;
    std::recursive_mutex m_mtxConnectStatus;

    friend class CAGVDiscovery;
    friend class CTcpClientServer;
};

#endif // CNETMANAGER_H
