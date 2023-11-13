#define GLOG_NO_ABBREVIATED_SEVERITIES
#include "NetManager.h"
#include "AGVDiscovery.h"
#include "TcpClientServer.h"
#include "agv_msg_def.h"
#include "uv.h"
#include "GlogWrapper.h"
#include <QThread>

SINGLETON_IMPLEMENT(CNetManager)


CNetManager::CNetManager()
{

}

bool CNetManager::Init()
{
    do{
        m_pControlTcpClientServer = new CTcpClientServer(ETCPClientServerType::Type_Control);
        if (!m_pControlTcpClientServer->Init())
        {
            break;
        }

        m_pFileTcpClientServer = new CTcpClientServer(ETCPClientServerType::Type_File);
        if (!m_pFileTcpClientServer->Init())
        {
            break;
        }

        m_pAGVDiscovery = new CAGVDiscovery(m_pControlTcpClientServer->m_pLoop);
        if (!m_pAGVDiscovery->Init())
        {
            break;
        }

        //开始扫描地图应答回调
        ListenMsg(Class_Data,  Type_Data_StartScanMap, std::bind(&CNetManager::OnIncRecvBuffer, this, std::placeholders::_1));
        //停止扫描地图应答回调
        ListenMsg(Class_Data,  Type_Data_StopScanMap, std::bind(&CNetManager::OnDecRecvBuffer, this, std::placeholders::_1));

        return true;
    }while(0);

    return false;
}

void CNetManager::Close()
{
    m_pFileTcpClientServer->NotifyExit();
    m_pControlTcpClientServer->NotifyExit();
    m_pFileTcpClientServer->WaitThreadExit();
    m_pControlTcpClientServer->WaitThreadExit();
}

bool CNetManager::SendRealTimeDataRes(string strAGVID)
{
    TMsgHeader tHeader;
    tHeader.uiDelimiter = DELIMITER;
    tHeader.shSrc = AGVCLIENT_ID;
    tHeader.shDes = atoi(strAGVID.c_str());
    tHeader.Class = Class_Control;
    //tHeader.SerialNum;//发送时计算
    tHeader.uiLen = 0;//不包含校验
    tHeader.Type = Type_Control_RealTimeData | TYPE_IS_ACK_FLAG;

    return SendAsyncDataWrap(&tHeader, nullptr, 0);
}

bool CNetManager::SendScanMapReq(string strAGVID, bool bStartScanMap)
{
    TMsgHeader tHeader;
    tHeader.uiDelimiter = DELIMITER;
    tHeader.shSrc = AGVCLIENT_ID;
    tHeader.shDes = atoi(strAGVID.c_str());
    tHeader.Class = Class_Data;
    //tHeader.SerialNum;//发送时计算
    tHeader.uiLen = 0;//不包含校验
    if (bStartScanMap)
    {
        tHeader.Type = Type_Data_StartScanMap;
    }
    else
    {
        tHeader.Type = Type_Data_StopScanMap;
    }

    return SendAsyncDataWrap(&tHeader, nullptr, 0);
}

bool CNetManager::SendMoveCtrl(string strAGVID, int iData)
{
    TMsgHeader tHeader;
    tHeader.uiDelimiter = DELIMITER;
    tHeader.shSrc = AGVCLIENT_ID;
    tHeader.shDes = atoi(strAGVID.c_str());
    tHeader.Class = Class_Control;
    //tHeader.SerialNum;//发送时计算
    tHeader.uiLen = 4;//不包含校验
    tHeader.Type = Type_Control_Move;

    return SendAsyncDataWrap(&tHeader, &iData, 4);
}

bool CNetManager::SendGoToDes(string strAGVID, TAGVGoToDes tDes)
{
    TMsgHeader tHeader;
    tHeader.uiDelimiter = DELIMITER;
    tHeader.shSrc = AGVCLIENT_ID;
    tHeader.shDes = atoi(strAGVID.c_str());
    tHeader.Class = Class_Control;
    tHeader.Type = Type_Control_GoStation;
    //tHeader.SerialNum;//发送时计算
    tHeader.uiLen = sizeof (tDes);//不包含校验

    return SendAsyncDataWrap(&tHeader, &tDes, tHeader.uiLen);
}

bool CNetManager::SendCancelGoToDes(string strAGVID)
{
    TMsgHeader tHeader;
    tHeader.uiDelimiter = DELIMITER;
    tHeader.shSrc = AGVCLIENT_ID;
    tHeader.shDes = atoi(strAGVID.c_str());
    tHeader.Class = Class_Control;
    tHeader.Type = Type_Control_StopGoStation;
    //tHeader.SerialNum;//发送时计算
    tHeader.uiLen = 0;//不包含校验

    return SendAsyncDataWrap(&tHeader, nullptr, 0);
}

bool CNetManager::SendTeleoperation(string strAGVID, int iData)
{
    TMsgHeader tHeader;
    tHeader.uiDelimiter = DELIMITER;
    tHeader.shSrc = AGVCLIENT_ID;
    tHeader.shDes = atoi(strAGVID.c_str());
    tHeader.Class = Class_Control;
    //tHeader.SerialNum;//发送时计算
    tHeader.uiLen = 4;//不包含校验
    tHeader.Type = Type_Control_Teleoperation;

    return SendAsyncDataWrap(&tHeader, &iData, 4);
}

void CNetManager::RegisterDisconnect(DisconnectCallBackFunc func)
{
    m_funcDisconnect = func;
}

void CNetManager::RegisterConnect(ConnectCallBackFunc func)
{
    m_funcConnect = func;
}

void CNetManager::ListenMsg(int iClass, int iType, ResponseCallFunc responseFunc)
{
    if (iClass==Class_Control && iType==Type_Control_Broadcast)
    {
        //广播的回调特殊处理
        m_pControlTcpClientServer->ListenMsg(iClass, iType, responseFunc);
        m_pFileTcpClientServer->ListenMsg(iClass, iType, responseFunc);
    }
    else
    {
        if (iClass == Class_Control)
        {
            m_pControlTcpClientServer->ListenMsg(iClass, iType, responseFunc);
        }
        else
        {
            m_pFileTcpClientServer->ListenMsg(iClass, iType, responseFunc);
        }

    }
}

void CNetManager::SendBroadcast()
{
    TMsgHeader tHeader;
    tHeader.uiDelimiter = DELIMITER;
    tHeader.shSrc = AGVCLIENT_ID;
    tHeader.Class = Class_Control;
    tHeader.Type = Type_Control_Broadcast;
    //tHeader.SerialNum;//发送时计算
    tHeader.uiLen = 0;//不包含校验

    m_pControlTcpClientServer->SendAsyncData(&tHeader, nullptr, 0);
}

bool CNetManager::SendMapData(std::string strAGVID, const char *pMapData, int ilen)
{
    TMsgHeader tHeader;
    tHeader.uiDelimiter = DELIMITER;
    tHeader.shSrc = AGVCLIENT_ID;
    tHeader.shDes = std::atoi(strAGVID.c_str());
    tHeader.Class = Class_Data;
    tHeader.Type = Type_Data_DownMapBagData;
    //tHeader.SerialNum;//发送时计算
    tHeader.uiLen = ilen;//不包含校验

    m_pFileTcpClientServer->SendUVData(&tHeader, (char*)pMapData, ilen);
}

//增加缓冲区大小
void CNetManager::OnIncRecvBuffer(TCallBackData *pData)
{
    auto itConnect = m_pFileTcpClientServer->m_mapAGVConnectInfo.find(std::to_string(pData->tHeader.shSrc));
    if (itConnect != m_pFileTcpClientServer->m_mapAGVConnectInfo.end())
    {
        itConnect->second.pRecvBuffer->AdjustSize(RECV_RING_BUFFER_MAX_SIZE);
    }
    else
    {
        assert(false);
    }

    DeleteCallBackData(pData);
}

//减小缓冲区大小
void CNetManager::OnDecRecvBuffer(TCallBackData *pData)
{
    auto itConnect = m_pFileTcpClientServer->m_mapAGVConnectInfo.find(std::to_string(pData->tHeader.shSrc));
    if (itConnect != m_pFileTcpClientServer->m_mapAGVConnectInfo.end())
    {
        itConnect->second.pRecvBuffer->AdjustSize(RECV_RING_BUFFER_SIZE);
    }
    else
    {
        assert(false);
    }

    DeleteCallBackData(pData);
}

//本函数有一个断开时，会被调用两次：
//第一次：检测断开的那个调用
//第二次：被上个断开强行断开的那个调用
void CNetManager::AGVDisconnect(string strAGVID, ETCPClientServerType type)
{
    std::lock_guard<std::recursive_mutex> lock(m_mtxConnectStatus);

    auto itAGV = m_mapAGVConnectStatus.find(strAGVID);
    if (itAGV == m_mapAGVConnectStatus.end())
    {
        assert(false);
        return;
    }

    if (type == ETCPClientServerType::Type_Control)
    {
        itAGV->second.bControlDisconnect = true;
    }
    else
    {
        itAGV->second.bDataDisconnect = true;
    }

    if (itAGV->second.bControlDisconnect && itAGV->second.bDataDisconnect)
    {
        m_mapAGVConnectStatus.erase(itAGV);
        m_funcDisconnect(strAGVID);
        GLOG_INFO << "AGVID:" << strAGVID.c_str() << "disconnect. two connection disconnect.";
    }
    else
    {
        //通知另一个连接断开
        TMsgHeader tHeader;
        tHeader.Class = Class_Internal;
        tHeader.Type = Type_Internal_ForceDisconnect;
        tHeader.shDes = std::atoi(strAGVID.c_str());
        tHeader.uiLen = 0;
       if (type == ETCPClientServerType::Type_Control)
       {
           m_pFileTcpClientServer->SendAsyncData(&tHeader, nullptr, 0);
       }
       else
       {
           m_pControlTcpClientServer->SendAsyncData(&tHeader, nullptr, 0);
       }
       GLOG_INFO << "AGVID:" << strAGVID.c_str()<< "disconnect. notify another connection disconnect.";
    }
}

void CNetManager::AGVConnect(void *pConnInfo, ETCPClientServerType type, bool bSucess)
{
    TAGVConnectInfo *pConn = (TAGVConnectInfo*)pConnInfo;
    std::lock_guard<std::recursive_mutex> lock(m_mtxConnectStatus);

    auto itUpdateStatus = [&](TAGVConnetcStatus &tData)
    {
        if (type == ETCPClientServerType::Type_Control)
        {
            if (bSucess)
            {
                tData.statusControl = AGVConnectStatus_Sucess;
            }
            else
            {
                tData.statusControl = AGVConnectStatus_Failure;
            }
        }
        else
        {
            if (bSucess)
            {
                tData.statusData = AGVConnectStatus_Sucess;
            }
            else
            {
                tData.statusData = AGVConnectStatus_Failure;
            }
        }
    };

    auto itAGV = m_mapAGVConnectStatus.find(pConn->strAGVID);
    if (itAGV == m_mapAGVConnectStatus.end())
    {
        TAGVConnetcStatus tStatus;
        tStatus.statusData = AGVConnectStatus_Invalid;
        tStatus.statusControl = AGVConnectStatus_Invalid;
        tStatus.bControlDisconnect = false;
        tStatus.bDataDisconnect = false;
        itUpdateStatus(tStatus);
        m_mapAGVConnectStatus.insert(make_pair(pConn->strAGVID, tStatus));

    }
    else
    {
        itUpdateStatus(itAGV->second);
        if (itAGV->second.statusControl==AGVConnectStatus_Failure || itAGV->second.statusData==AGVConnectStatus_Failure){
            m_funcConnect(itAGV->first, pConn->strAGVIP, false, pConn->tBroadcResData);
            m_mapAGVConnectStatus.erase(itAGV);//删除自己
            //清除连接
            std::string strAGVID = pConn->strAGVID;
            m_pFileTcpClientServer->ClearConnectInfo(strAGVID);
            m_pControlTcpClientServer->ClearConnectInfo(strAGVID);
            GLOG_INFO << "agvid:" << strAGVID.c_str() << "two connection disconnect";
        }else if (itAGV->second.statusControl==AGVConnectStatus_Sucess && itAGV->second.statusData==AGVConnectStatus_Sucess){
            m_funcConnect(itAGV->first, pConn->strAGVIP, true, pConn->tBroadcResData);
        }
    }
}

bool CNetManager::SendAsyncDataWrap(TMsgHeader *pHeader, void *pData, int ilen)
{
    auto itAGV = m_mapAGVConnectStatus.find(std::to_string(pHeader->shDes));
    if (itAGV == m_mapAGVConnectStatus.end())
    {
        return false;
    }

    if (!(itAGV->second.statusControl==AGVConnectStatus_Sucess &&  itAGV->second.statusData==AGVConnectStatus_Sucess))
    {
        return false;
    }

    if (pHeader->Class == Class_Control)
    {
        return m_pControlTcpClientServer->SendAsyncData(pHeader, pData, ilen);
    }
    else
    {
        return m_pFileTcpClientServer->SendAsyncData(pHeader, pData, ilen);
    }

    return true;
}



