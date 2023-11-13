//TCP客户端管理
#ifndef CTCPSERVER_H
#define CTCPSERVER_H
#define GLOG_NO_ABBREVIATED_SEVERITIES
#include <string>
#include <thread>
#include <functional>
#include <map>
#include <functional>
#include <unordered_map>
#include <list>
#include <set>
#include "uv.h"
#include "agv_msg_def.h"
#include "LockFreeRingBuffer.h"
#include "NetManager.h"
#include "GoerLockQueue.h"

//发送超时定时器包装
typedef struct TTimerSendWrapper{
    bool bDelete; //是否已关闭
    uv_timer_t uv_timer; //
} TTimerSendWrapper;

//某个AGV连接信息
typedef struct TAGVConnectInfo{
    //AGVID
    std::string strAGVID;
    //AGVIP
    std::string strAGVIP;
    //广播应答数据暂存
    TBroadcastResData tBroadcResData;

    uv_tcp_t client; //tcp句柄类型，该data存放的是本连接信息TAGVConnectInfo*
    uv_connect_t connect_t; //请求类，连接请求，data存放本连接信息TAGVConnectInfo*
    uv_write_t write_t; //请求类，写入请求，该data存放的是本连接信息TAGVConnectInfo*

    //发送序列号
    unsigned int uiSQ;

    //检测是否发送超时的消息CT队列。key:CT组合，value:定时器列表。
    //这里key只考虑CT组合，后期可考虑CTSN组合，因为连续发送相同CT的消息只删除头部的
    //[2021-7-23]去掉发送超时检测，只检测心跳，tcp可以这样
    //std::map<int, std::list<TTimerSendWrapper> > mapCheckSendTimeOut; //uv_timer_t的data存放本连接信息TAGVConnectInfo*

    //连接是否无数据超时检测定时器，!!只针对控制连接！！。
    uv_timer_t timerNoData;//data存放本连接信息TAGVConnectInfo*

    //连接关闭计数器，在控制连接中，超时检测定时器回调关闭计数+1，主动关闭回调+1，当检测到计数为2时，删除该连接
    //这是因为两个回调顺序不可控，提前删除连接中的定时器数据导致崩溃
    int iCloseCount;

    //数据接收缓冲区
    CLockFreeRingBuffer *pRecvBuffer;

    //所属server指针
    CTcpClientServer *pServer;

} TAGVConnectInfo;

class CTcpClientServer
{
public:
    CTcpClientServer(ETCPClientServerType type);

    bool Init();

    /**
     * @brief SendAsyncData : send data async
     * @param pHeader : the header of send data.
     * @param pData : the send actual data.
     * @param ilen : the length of send actual data.
     */
    bool SendAsyncData(TMsgHeader *pHeader, void *pData, int ilen);

    /**
     * @brief SendUVData : send data by uv_write, it must call this function in communicate thread.
     * @param pHeader
     * @param pData
     * @param ilen
     */
    void SendUVData(TMsgHeader *pHeader, void *pData, int ilen);

    /**
     * @brief ListenMsg
     * @param iClass
     * @param iType
     * @param responseFunc
     */
    void ListenMsg(int iClass, int iType, ResponseCallFunc responseFunc);

    /**
     * @brief ClearConnectInfo : 清除指定的连接
     * @param strAGVID
     */
    void ClearConnectInfo(std::string strAGVID);

    /**
     * @brief NotifyExit : 通知退出通讯线程
     */
    void NotifyExit();


private:
    //AGV连入
    void AGVLogin(unsigned short shID, TBroadcastResInter *ptData);
    //线程函数
    void ThreadCallBack();
    //向上层发送回调数据，pData可以没有，如果有必须是new出来的
    void SendMsgCallBack(TMsgHeader tHeader, void *pData);
    //发送广播数据
    void SendBroadcastData();
    //AGV断开连接登记
    void AGVDisconnect(std::string strAGVID);
    //AGV连接登记
    void AGVConnect(TAGVConnectInfo *pConnInfo, bool bSucess);
    //强制断开连接
    void ForceDisconnect(std::string strAGVID);
    //等待线程退出
    void WaitThreadExit();

private:
    //连接端口
    int m_iConnectPort;
    //uv_loop句柄，一个连接一个，因为数据类消息会有循环发送文件占用连接的过程。
    uv_loop_t* m_pLoop;
    //通讯线程
    std::thread *m_pthread;
    //UI->通信模块，两个线程间的异步数据发送缓冲队列,存储类型为TSendQAsyncData+data
    CGoerLockQueue<TSendQAsyncData> m_AsyncSendQ;
    //连接对象列表 key:agvid
    std::map<std::string, TAGVConnectInfo> m_mapAGVConnectInfo;
    //key:  response class left shift 4 & type. value:response func list.
    std::unordered_map<int, std::list<ResponseCallFunc > > m_mapFuncCB; //暂未加锁，仅限于初始化时调用
    //服务器类型
    ETCPClientServerType m_type;
    //uv异步操作对象--发送队列
    uv_async_t m_async;
    //uv异步操作对象--退出
    uv_async_t m_async_exit;
    //发送时申请的内存数据块，发送完成后删除,这里假定是按顺序入列表的
    std::map<uv_write_t*, std::list<char *> > m_mapSendMem;

    friend void async_cb(uv_async_t* handle);
    friend void connect_cb(uv_connect_t* req, int status);
    friend void close_cb(uv_handle_t* handle);
    friend void tcp_read_cb(uv_stream_t* stream, ssize_t nread, const uv_buf_t* buf);
    friend void write_cb(uv_write_t* req, int status);
    friend void nodata_timer_cb(uv_timer_t* handle);
    friend void close_timers(TAGVConnectInfo *pConnect);
    friend void async_exit_cb(uv_async_t* handle);
    friend void timer_close_cb(uv_handle_t* handle);
    friend class CNetManager;
    friend class CAGVDiscovery;

};

#endif // CTCPSERVER_H
