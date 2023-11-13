#include "AGVDiscovery.h"
#include "ConfigManager.h"
#include "GlogWrapper.h"
#include "NetManager.h"
#include "TcpClientServer.h"
#include "GoerToolUtile.h"

static void alloc_cb(uv_handle_t* handle, size_t suggested_size, uv_buf_t* buf){
    buf->base = (char*)malloc(suggested_size);
    buf->len = suggested_size;
}

//接收数据回调
void udp_recv_cb(uv_udp_t* handle, ssize_t nread, const uv_buf_t* buf, const struct sockaddr* addr, unsigned flags){
    if (nread < 0){
        uv_close((uv_handle_t*)handle, NULL);
        free(buf->base);//记得释放内存
        return;
    }

#if defined(Debug) || defined(QT_DEBUG)
    GLOG_INFO << "udp recv:" << CGoerToolUtile::HexToString(buf->base, nread).toStdString().c_str();
#endif

    TMsgHeader *pHeader = (TMsgHeader*)buf->base;
    GLOG_INFO << "recv broadcast :sn" << pHeader->SerialNum << "class:" << pHeader->Class << "type:" << pHeader->Type;

    //agv上线
    CAGVDiscovery *pAGV = (CAGVDiscovery*)handle->data;
    TBroadcastResInter tData;
    memcpy(&tData.tData, buf->base+sizeof(TMsgHeader), pHeader->uiLen);
    const struct sockaddr_in *psock = (const struct sockaddr_in*)addr;
    uv_inet_ntop(AF_INET, &psock->sin_addr.S_un.S_addr, tData.chIP, sizeof(tData.chIP));

    pAGV->AGVLogin(pHeader->shSrc, tData);

    free(buf->base);//记得释放内存
}

//发送完成回调
static void udp_send_cb(uv_udp_send_t* req, int status){

}

CAGVDiscovery::CAGVDiscovery(uv_loop_t *ploop)
{
    m_puvloop = ploop;
}

bool CAGVDiscovery::Init()
{
    int iBindPort, iClientRecvPort;
    std::string strBindIP;
    //读取配置
    iBindPort = CConfigManager::GetInstance()->GetValueInt("broadcast_server_bind_port");
    iClientRecvPort = CConfigManager::GetInstance()->GetValueInt("broadcast_agv_recv_port");
    strBindIP = CConfigManager::GetInstance()->GetValueString("broadcast_host_ip").toStdString();

    //初始化广播套接字
    m_puvUDPSocket = (uv_udp_t*)malloc(sizeof (uv_udp_t));
    if (0 != uv_udp_init(m_puvloop, m_puvUDPSocket)){
        return false;
    }
    struct sockaddr_in broadcast_addr;
    uv_ip4_addr(strBindIP.c_str(), iBindPort, &broadcast_addr); //构造send_addr,注意这里UDP广播只能在一个子网内，电脑有有线和无线时，必须指明绑定哪个IP
    uv_udp_bind(m_puvUDPSocket, (const struct sockaddr*)&broadcast_addr, UV_UDP_REUSEADDR);//绑定并初始化socket
    uv_udp_set_broadcast(m_puvUDPSocket, 1);//设置广播模式

    m_psend_addr = (struct sockaddr_in*)malloc(sizeof (struct sockaddr_in));
    uv_ip4_addr("255.255.255.255", iClientRecvPort, m_psend_addr);

    m_puvUDPSocket->data = this;
    uv_udp_recv_start(m_puvUDPSocket, alloc_cb, udp_recv_cb);//绑定接收回调，收到数据会调用alloc_cb分配内存

    m_sendBuff.base = (char*)malloc(PACK_SIZE);
    m_sendBuff.len = sizeof (TMsgHeader);
    TMsgHeader *pHeader = (TMsgHeader*)m_sendBuff.base;
    pHeader->shSrc = AGVCLIENT_ID;
    pHeader->shDes = 0x0000;
    pHeader->Class = Class_Control;
    pHeader->Type = Type_Control_Broadcast;
    pHeader->uiLen = 0;
    pHeader->SerialNum = 0;
    pHeader->uiDelimiter = DELIMITER;
    //CRC32校验 TODO

    return true;

}

#include <winsock2.h>

void CAGVDiscovery::SendBroadcast()
{
#if defined(Debug) || defined(QT_DEBUG)
    GLOG_INFO << "udp send:" << CGoerToolUtile::HexToString(m_sendBuff.base, m_sendBuff.len).toStdString().c_str();
#endif
    uv_udp_send(&send_req, m_puvUDPSocket, &m_sendBuff, 1, (const struct sockaddr*)m_psend_addr, udp_send_cb);

    //测试udp广播，发现只能在一个子网内使用
//    struct sockaddr_in own_addr;
//    int sock = socket(AF_INET, SOCK_DGRAM, 0);
//    bool fBroadcast = TRUE;
//    setsockopt(sock,SOL_SOCKET,SO_BROADCAST,(char*)&fBroadcast,sizeof(fBroadcast));//设置套接字类型
//    own_addr.sin_family = AF_INET;
//    own_addr.sin_port = htons(20000);
//    own_addr.sin_addr.s_addr = inet_addr("0.0.0.0")/*htonl(INADDR_ANY)*/;
//    int i = ::bind(sock, (struct sockaddr *)&own_addr, sizeof(own_addr));

//    struct sockaddr_in addrto;
//    addrto.sin_family=AF_INET;
//    addrto.sin_addr.s_addr=inet_addr("255.255.255.255");
//    addrto.sin_port=htons(20001);
//    int iret = sendto(sock,m_sendBuff.base,m_sendBuff.len,0,(sockaddr*)&addrto,sizeof (addrto));//向广播地址发布消息
//    int ierr = GetLastError();
//    LOG_DEBUG("sendto GetLastError:%d", ierr);

//    char buffer[1024];
//    struct sockaddr_in addrfrom;
//    int ilen = sizeof (addrfrom);
//    while(1)
//    {
//        int ii = recvfrom(sock, buffer, 1024, 0, (sockaddr*)&addrfrom, &ilen);
//        if (ii > 0)
//        {
//            LOG_DEBUG("rev :%s", CGoerToolUtile::HexToString(buffer, ii).toStdString().c_str());
//        }
//        else {
//            ierr = GetLastError();
//            LOG_DEBUG("rev GetLastError:%d", ierr);
//        }
//        Sleep(100);
//    }

}

void CAGVDiscovery::AGVLogin(unsigned short shID, TBroadcastResInter tData)
{
    TMsgHeader tHeader;
    tHeader.Class = Class_Internal;
    tHeader.Type = Type_Internal_BroadcastResponse;
    tHeader.uiLen = sizeof (TBroadcastResInter);
    tHeader.shSrc = shID;


    CNetManager::GetInstance()->m_pControlTcpClientServer->SendAsyncData(&tHeader, (void*)&tData, sizeof(tData));
    CNetManager::GetInstance()->m_pFileTcpClientServer->SendAsyncData(&tHeader, (void*)&tData, sizeof(tData));
}

