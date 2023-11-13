/*
 *libuv总结与踩坑：
 * ① 定时器uv_timer_t，在关闭时必须调用uv_close()来关闭，同时定时器对象必须在uv_close()传入的回调中释放，否则会崩溃；
 * ② uv_close()可以关闭各种handle，必须用uv_close关闭的handle可以看看本函数的源码。
 * ③ endgame()顺序导致的崩溃问题：read为负值，回调中会调用uv_close()来关闭tcp连接，但由于此时read()回调没执行完，
 * uv_close()并不会将本tcp handle对象添加到endgame列表，此时回调中调用close_timers(),导致endgame()对象链表中先加上了定时器，等read()回调
 * 返回时，加上了tcp handle对象到endgame中，注意这个endgame链表是往头部插入的，所以导致uv底层会先释放tcp对象，再释放timers对象，此时在释放tcp对象的
 * close回调中，已经删除了timers对象，结果uv释放timers对象时，已经没有了，崩溃！（此方法没法通过移动close_timers()位置解决不了，不同场景调用顺序不一致）
 * 【解决办法】：添加一个计数器，删除后+1，然后判断总数
 *
*/
#include "TcpClientServer.h"
#include "ConfigManager.h"
#include "GlogWrapper.h"
#include "AGVManager.h"
#include "AGVDiscovery.h"
#include "GoerToolUtile.h"
#include <QDebug>

void timer_close_cb(uv_handle_t* handle)
{
    TAGVConnectInfo *pConnect = (TAGVConnectInfo*)handle->data;
    qDebug() << "timer_close_cb" << (int)pConnect->pServer->m_type ;

    //删除当前连接
    for (auto &it : pConnect->pServer->m_mapAGVConnectInfo)
    {
        if (&it.second.timerNoData == (uv_timer_t*)handle)
        {
            it.second.iCloseCount++;
            if (2 == it.second.iCloseCount)
            {
                //其中的定时器会在此销毁
                pConnect->pServer->m_mapAGVConnectInfo.erase(it.first);
                break;
            }
        }
    }
}

//关闭该连接的所有定时器，在调用uv_close()tcp连接之前必须调用这个
void close_timers(TAGVConnectInfo *pConnect)
{
    if(pConnect->pServer->m_type == ETCPClientServerType::Type_Control)
    {
        uv_close((uv_handle_t*)&pConnect->timerNoData, timer_close_cb); //内部调用uv_timer_stop()
    }

}

static void alloc_cb(uv_handle_t* handle, size_t suggested_size, uv_buf_t* buf)
{
    buf->base = (char*)malloc(suggested_size);
    buf->len = suggested_size;
    //handle的data不能随便动
}

//stream为uv_tcp_t*
void close_cb(uv_handle_t* handle)
{
    uv_tcp_t *ptcp = (uv_tcp_t*)handle;
    TAGVConnectInfo *pConnect = (TAGVConnectInfo*)ptcp->data;
    qDebug() << "close_cb" << (int)pConnect->pServer->m_type ;

    for (auto &it : pConnect->pServer->m_mapAGVConnectInfo)
    {
        if (&it.second.client == ptcp)
        {

            //标记该连接断开
            pConnect->pServer->AGVDisconnect(it.second.strAGVID);

            //删除对象
            delete it.second.pRecvBuffer;

            //其中文件连接的定时器会在此销毁
            if(pConnect->pServer->m_type == ETCPClientServerType::Type_File)
            {
                pConnect->pServer->m_mapAGVConnectInfo.erase(it.first);
            }
            else
            {
                pConnect->iCloseCount++;
                if (2 == pConnect->iCloseCount)
                {
                    //其中的定时器会在此销毁
                    pConnect->pServer->m_mapAGVConnectInfo.erase(it.first);
                }
            }

            break;
        }
    }
}

//stream为uv_tcp_t*
void tcp_read_cb(uv_stream_t* stream, ssize_t nread, const uv_buf_t* buf)
{
    TAGVConnectInfo *pConnect = (TAGVConnectInfo*)stream->data;

    if (nread < 0)
    {
        if ((int)pConnect->pServer->m_type == 0)
        {
            int a= 1;
            int b = a;
        }

        close_timers(pConnect);
        uv_close((uv_handle_t*)stream, close_cb);
        free(buf->base);
        qDebug() << "nread < 0" << (int)pConnect->pServer->m_type ;
        return;
    }

#if defined(Debug) || defined(QT_DEBUG)
    //LOG_DEBUG("tcp type %d recv:%s", pConnect->pServer->m_type, CGoerToolUtile::HexToString(buf->base, nread).toStdString().c_str());
#endif

    uv_tcp_t *ptcp = (uv_tcp_t*)stream;
    for (auto &it : pConnect->pServer->m_mapAGVConnectInfo)
    {
        if (&it.second.client == ptcp)
        {
            //数据放入环形缓冲区
            if (!it.second.pRecvBuffer->PutData((const unsigned char*)buf->base, nread))
            {
                GLOG_WARN << "agvid:" << it.second.strAGVID.c_str() << "recv ringbuffer overflow!";
            }
            else
            {
                //循环读取缓冲区数据
                TMsgHeader tHeader;
                unsigned long long ilen = it.second.pRecvBuffer->DataLen();
                while (ilen >= sizeof(TMsgHeader))
                {
                    //查看数据
                    it.second.pRecvBuffer->PeekData((unsigned char *)&tHeader, sizeof(TMsgHeader));
                    //查看数据有效性
                    if (tHeader.uiDelimiter != DELIMITER)
                    {
                        //数据无效，取出头部扔掉
                        it.second.pRecvBuffer->GetData((unsigned char *)&tHeader, sizeof(TMsgHeader));
                        continue;
                    }
                    //取出数据
                    if (ilen >= sizeof (TMsgHeader) + tHeader.uiLen)
                    {
                        //取出头
                        it.second.pRecvBuffer->GetData((unsigned char *)&tHeader, sizeof(TMsgHeader));

                        //取出数据体
                        char *pbuff = nullptr;
                        if (tHeader.uiLen != 0)
                        {
                            pbuff = (char*)malloc(tHeader.uiLen);
                            it.second.pRecvBuffer->GetData((unsigned char *)pbuff, tHeader.uiLen);
                        }

                        if (tHeader.Class==Class_Control && tHeader.Type==Type_Control_GlobalPath)
                        {
                            int a = 1;
                            int b = a+1;
                            GLOG_INFO << ("recv one map data..");
                        }

                        //CRC 校验 todo

                        //重置有无数据定时器
                        if (pConnect->pServer->m_type == ETCPClientServerType::Type_Control)
                        {
                            uv_timer_again(&it.second.timerNoData);
                        }

                        //应答消息
                        if ((tHeader.Type & TYPE_IS_ACK_FLAG) == TYPE_IS_ACK_FLAG)
                        {
                            tHeader.Type = (tHeader.Type&(~TYPE_IS_ACK_FLAG)); //摘掉最高位
                        }

                        //发送响应给上层
                        pConnect->pServer->SendMsgCallBack(tHeader, pbuff);

//                        GLOG_INFO << "recv one intact data: agvid:%x" << tHeader.shSrc << "class:"
//                                  <<tHeader.Class << "type:" << tHeader.Type;

                    }
                    else
                    {
                        break; //没收齐一个包，下次继续收
                    }
                    ilen = it.second.pRecvBuffer->DataLen();
                }
            }
        }
        break;
    }
    free(buf->base);
}

//本回调执行的时机：是压倒缓冲区完成，不是对端收到。
void write_cb(uv_write_t* req, int status)
{
    TAGVConnectInfo *pConnect = (TAGVConnectInfo*)req->data;

    if (status != 0)
    {
        GLOG_WARN << "write_cb not sucess:" << uv_err_name(status);
    }

    //释放发送时申请的内存
    auto itValue = pConnect->pServer->m_mapSendMem.find(req);
    if (itValue != pConnect->pServer->m_mapSendMem.end())
    {
        //LOG_DEBUG("free send mem address:%x", itValue->second.front());
        free(itValue->second.front());
        itValue->second.pop_front();
        if (itValue->second.empty())
        {
            pConnect->pServer->m_mapSendMem.erase(itValue);
        }
    }
    else
    {
        assert(false);
    }
}

void connect_cb(uv_connect_t* req, int status)
{
    TAGVConnectInfo *pConnect = (TAGVConnectInfo*)req->data;

    if (status != 0)
    {
        GLOG_INFO << "connect_cb err:" << uv_err_name(status);
        pConnect->pServer->AGVConnect(pConnect, false);
    }
    else
    {
        GLOG_INFO << ("connect_cb sucessful");
        pConnect->pServer->AGVConnect(pConnect, true);
        uv_read_start((uv_stream_t*)req->handle, alloc_cb, tcp_read_cb);//开始监听数据
    }
}

//控制连接没有数据超时回调
void nodata_timer_cb(uv_timer_t* handle)
{
    TAGVConnectInfo *pConnect = (TAGVConnectInfo*) handle->data;

    close_timers(pConnect);//关闭连接检测定时器
    uv_close((uv_handle_t*)&pConnect->client, close_cb);

    GLOG_INFO << ("nodata time out, disconnect!");
}

//异步通知退出通讯线程
void async_exit_cb(uv_async_t* handle){
    uv_stop(handle->loop);
}

/**
 * @brief 当uv_async_send调用后，在loop中异步回调的接口
 * @param handle
 */
void async_cb(uv_async_t* handle)
{
    CTcpClientServer *pServer = (CTcpClientServer*)handle->data;
    TSendQAsyncData tData;

    while (pServer->m_AsyncSendQ.size() != 0)
    {
        pServer->m_AsyncSendQ.get(0, tData);
        pServer->m_AsyncSendQ.pop_front();

        //发送数据
        if (tData.tHeader.Class==Class_Control && tData.tHeader.Type==Type_Control_Broadcast)
        {
            pServer->SendBroadcastData();
        }
        else if(tData.tHeader.Class==Class_Internal && tData.tHeader.Type==Type_Internal_ForceDisconnect)
        {
            pServer->ForceDisconnect(std::to_string(tData.tHeader.shDes)); //强行断开连接
        }
        else if(tData.tHeader.Class==Class_Internal && tData.tHeader.Type==Type_Internal_BroadcastResponse)
        {
            pServer->AGVLogin(tData.tHeader.shSrc, (TBroadcastResInter*)tData.pData);
        }
        else
        {
            //发送数据
            pServer->SendUVData(&tData.tHeader, tData.pData, tData.tHeader.uiLen);
            //qDebug() << tData.tHeader.Class << "  " << tData.tHeader.Type << tData.tHeader.uiLen;
        }
        free(tData.pData);
    }
}

CTcpClientServer::CTcpClientServer(ETCPClientServerType type)
{
    m_type = type;
}

bool CTcpClientServer::Init()
{
    do{
        //loop
        m_pLoop = (uv_loop_t*)malloc(sizeof(uv_loop_t));
        if (m_pLoop == NULL)
        {
            break;
        }
        if (uv_loop_init(m_pLoop))
        {
          free(m_pLoop);
          break;
        }

        //启动线程
        m_pthread = new std::thread([&](){this->ThreadCallBack();});

        m_async.data = this;
        if(0 != uv_async_init(m_pLoop, &m_async, async_cb))
        {
            break;
        }

        if(0 != uv_async_init(m_pLoop, &m_async_exit, async_exit_cb))
        {
            break;
        }

        if (m_type == ETCPClientServerType::Type_Control)
        {
            m_iConnectPort = CConfigManager::GetInstance()->GetValueInt("agv_control_port");
        }
        else
        {
            m_iConnectPort = CConfigManager::GetInstance()->GetValueInt("agv_file_port");
        }

        return true;
    }while(0);

    return false;
}

void CTcpClientServer::AGVLogin(unsigned short shID, TBroadcastResInter *ptData)
{
    std::string strAGVID = std::to_string(shID);
    std::string strAGVIP = ptData->chIP;

    auto itAGV = m_mapAGVConnectInfo.find(strAGVID);
    if (itAGV != m_mapAGVConnectInfo.end())
    {
        GLOG_WARN << "agvid:" << shID << "have connected, agvlogin return!";
        return; //已经在线了
    }

    sockaddr_in addr_agv;

    //命令口连接
    auto itRet = m_mapAGVConnectInfo.insert(make_pair(strAGVID, TAGVConnectInfo()));
    itRet.first->second.tBroadcResData = ptData->tData;
    itRet.first->second.uiSQ = 0;
    itRet.first->second.pRecvBuffer = new CLockFreeRingBuffer(RECV_RING_BUFFER_SIZE);
    itRet.first->second.pRecvBuffer->Init();
    itRet.first->second.connect_t.data = &itRet.first->second;
    itRet.first->second.client.data = &itRet.first->second;
    if (m_type == ETCPClientServerType::Type_Control)
    {
        itRet.first->second.iCloseCount = 0;
        uv_timer_init(m_pLoop, &itRet.first->second.timerNoData);
        itRet.first->second.timerNoData.data = &itRet.first->second;
        uv_timer_start(&itRet.first->second.timerNoData, nodata_timer_cb, NODATA_OVERTIME, NODATA_OVERTIME);
    }
    itRet.first->second.strAGVID = strAGVID;
    itRet.first->second.strAGVIP = strAGVIP;
    itRet.first->second.write_t.data = &itRet.first->second;
    itRet.first->second.pServer = this;
    uv_tcp_init(m_pLoop, &itRet.first->second.client);
    uv_ip4_addr(strAGVIP.c_str(), m_iConnectPort, &addr_agv);
    uv_tcp_nodelay(&itRet.first->second.client, 1); //设置no_delay，关闭nagle算法，避免发小包延迟
    uv_tcp_connect(&itRet.first->second.connect_t, &itRet.first->second.client, (const struct sockaddr*)&addr_agv, connect_cb);//异步连接，超时时间暂不可控，默认20s左右超时报错

}

bool CTcpClientServer::SendAsyncData(TMsgHeader *pHeader, void *pData, int ilen)
{
    TSendQAsyncData tData;
    assert((int)pHeader->uiLen == ilen);

    tData.tHeader = *pHeader;
    if (ilen == 0)
    {
        tData.pData = nullptr;
    }
    else
    {
        tData.pData = malloc(ilen);
        memcpy(tData.pData, pData, ilen);
    }

    m_AsyncSendQ.push_back(tData);
    uv_async_send(&m_async); //发送异步消息，async_cb真正回调，保证发送过程都在同一个线程中(内部调用postcompletionport，导致iocp等待返回了该重叠IO)
    return true;
}


void CTcpClientServer::SendUVData(TMsgHeader *pHeader, void *pData, int ilen)
{
    assert((int)pHeader->uiLen == ilen);

    std::string strAGVID = std::to_string(pHeader->shDes);
    auto itAGV = m_mapAGVConnectInfo.find(strAGVID);
    if (itAGV == m_mapAGVConnectInfo.end())
    {
        GLOG_WARN << "SendSyncData not find agvid:" << strAGVID.c_str();
        return;
    }
    //序列号
    pHeader->SerialNum = itAGV->second.uiSQ++;
    //计算CRC
    //拷贝到发送缓冲区
    uv_buf_t buf;
    buf.len = sizeof (TMsgHeader)+pHeader->uiLen;
    buf.base = (char*)malloc(buf.len);//TODO 未加CRC长度
    m_mapSendMem[&itAGV->second.write_t].push_back(buf.base);
    //LOG_DEBUG("malloc send mem address:%x", buf.base);
    memcpy(buf.base, pHeader, sizeof (TMsgHeader));
    memcpy(buf.base+sizeof (TMsgHeader), pData, pHeader->uiLen);

//    GLOG_INFO << "send data, agvid:" << pHeader->shDes << "class:" << pHeader->Class
//              << "type:" << pHeader->Type << "(" << pHeader->Type << ")";
    uv_write(&itAGV->second.write_t, (uv_stream_t*)&itAGV->second.client, &buf, 1, write_cb);

}

void CTcpClientServer::SendMsgCallBack(TMsgHeader tHeader, void *pData)
{
    TCallBackData * pCBData = new TCallBackData;
    pCBData->tHeader = tHeader;
    pCBData->pData = pData;

    //发送UI数据，注意pData必须是new出来
    auto listValue = m_mapFuncCB.find(MAKE_CLASSTYPE_KEY(tHeader.Class, tHeader.Type));
    //测试
    if (tHeader.Class==Class_Data && tHeader.Type==Type_Data_ReportGridMap)
    {
        int a = 1;
        int b = a+1;
    }
    if (listValue != m_mapFuncCB.end())
    {
        pCBData->iref = listValue->second.size();
        for (auto &itValue : listValue->second)
        {
            itValue(pCBData);
        }
    }
//    else
//    {
//        assert(false);
//    }
}

void CTcpClientServer::SendBroadcastData()
{
    CNetManager::GetInstance()->m_pAGVDiscovery->SendBroadcast();
}

void CTcpClientServer::AGVDisconnect(string strAGVID)
{
    CNetManager::GetInstance()->AGVDisconnect(strAGVID, m_type);
}

void CTcpClientServer::AGVConnect(TAGVConnectInfo *pConnInfo, bool bSucess)
{
    CNetManager::GetInstance()->AGVConnect(pConnInfo, m_type, bSucess);
}


void CTcpClientServer::ListenMsg(int iClass, int iType, ResponseCallFunc responseFunc)
{
    static std::thread::id id = std::this_thread::get_id();
    assert(id == std::this_thread::get_id()); //确保本函数为同一个线程调用，保证m_mapFuncCB不需要加锁

    int ikey = MAKE_CLASSTYPE_KEY(iClass, iType);
    auto itKey = m_mapFuncCB.find(ikey);
    if (itKey != m_mapFuncCB.end())
    {
        itKey->second.push_back(responseFunc);
    }
    else
    {
        std::list<ResponseCallFunc> listTemp = {responseFunc};
        m_mapFuncCB.insert(make_pair(ikey, listTemp));
    }
}

void CTcpClientServer::ClearConnectInfo(string strAGVID)
{
    auto itAGVConn = m_mapAGVConnectInfo.find(strAGVID);
    if (itAGVConn != m_mapAGVConnectInfo.end())
    {
        m_mapAGVConnectInfo.erase(itAGVConn);
    }
    else
    {
        assert(false);
    }
}

void CTcpClientServer::NotifyExit()
{
    uv_async_send(&m_async_exit);
}

void CTcpClientServer::ForceDisconnect(string strAGVID)
{
    auto itAGV = m_mapAGVConnectInfo.find(strAGVID);
    if (itAGV != m_mapAGVConnectInfo.end())
    {
        uv_handle_t *ph = (uv_handle_t*)&itAGV->second.client;
        if (!uv_is_active(ph)) //Debug模式下，该handle可能提前被设置为了无效，此时需手动清除数据
        {
            //标记该连接断开
            AGVDisconnect(itAGV->second.strAGVID);
            //删除对象
            delete itAGV->second.pRecvBuffer;
            m_mapAGVConnectInfo.erase(itAGV->first);
            return;
        }

        close_timers(&itAGV->second);
        uv_close(ph, close_cb);

        GLOG_INFO << "agvid:" << strAGVID.c_str() << "ForceDisconnect sucess";
    }
    else
    {
        GLOG_INFO << "agvid:" << strAGVID.c_str() << "ForceDisconnect, but not exist.";
    }
}

void CTcpClientServer::WaitThreadExit()
{
    m_pthread->join();
}

void CTcpClientServer::ThreadCallBack()
{
    while (1) {
        uv_run(m_pLoop, UV_RUN_DEFAULT); //内部无限循环
        break;
    }
}
