//广播发现AGV
#ifndef CAGVDISCOVERY_H
#define CAGVDISCOVERY_H
#define GLOG_NO_ABBREVIATED_SEVERITIES
#include "uv.h"
#include "agv_msg_def.h"

class CAGVDiscovery
{
public:
    CAGVDiscovery(uv_loop_t *ploop);

    /**
     * @brief Init
     * @return
     */
    bool Init();

    /**
     * @brief send broadcast data
     */
    void SendBroadcast();

private:
    void AGVLogin(unsigned short shID, TBroadcastResInter tData);

private:
    //uv_loop循环
    uv_loop_t *m_puvloop;
    //udp句柄
    uv_udp_t *m_puvUDPSocket;
    //AGV接收广播包的端口
    struct sockaddr_in *m_psend_addr;
    //广播数据报文
    uv_buf_t m_sendBuff;
    uv_udp_send_t send_req;

    friend class CTcpClientServer;
    friend void udp_recv_cb(uv_udp_t* handle, ssize_t nread, const uv_buf_t* buf, const struct sockaddr* addr, unsigned flags);
};

#endif // CAGVDISCOVERY_H
