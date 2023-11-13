/******************************************************************************
* Copyright (c) 2016 歌尔股份有限公司
* All rights reserved.
*
*文件名称：GoerLockQueue.h
*摘    要：内部提供锁操作的队列，外部无需加锁。
    简化push和pop数据时的加锁处理；等待线程调用wait时会进行
    线程阻塞，降低cpu消耗
    ①线程中先wait()
    ②再if(!.get(0, tSave)
        {
            continue;
        }
    ③pop
    或者：
    while (.size() != 0){
        .get(0, tData);
        .pop_front();
    }
*
* 版　本：1.0
* 创建人：bob.li
* 日　期：2016.09.08
*
******************************************************************************/
#pragma once
#include <QMutex>
#include <QWaitCondition>
#include <deque>
#include "commonlibrary_global.h"
using namespace std;

#include "gr_type.h"

template<class T>
class COMMONLIBRARYSHARED_EXPORT CGoerLockQueue
{
public:
    CGoerLockQueue(void);
    ~CGoerLockQueue(void);

    //接口

    bool push_back(const T &c_t);   //尾部添加
    bool push_front(const T &c_t);  //头部添加

    void pop_back();    //尾部弹出
    void pop_front();   //头部弹出

    void wait();        //等待事件

    bool get(GR_INT i, T &t);//注意返回的是拷贝的内容,false时队列为空

    GR_INT size();//获取大小

    void clear();//清除队列数据
 
private:
    deque<T> m_dequeue;
    QMutex m_mtxQueue;               //访问队列互斥控制

    QMutex m_mtxWait;                //等待条件控制
    QWaitCondition m_waitCondition;  //事件控制
};

template<class T>
GR_INT CGoerLockQueue<T>::size()
{
    return (GR_INT)m_dequeue.size();
}

template<class T>
inline void CGoerLockQueue<T>::clear()
{
    m_dequeue.clear();
}

template<class T>
CGoerLockQueue<T>::CGoerLockQueue(void)
{
}

template<class T>
CGoerLockQueue<T>::~CGoerLockQueue(void)
{
}

template<class T>
bool CGoerLockQueue<T>::push_back(const T &c_t)
{
    bool bret = true;
    m_mtxQueue.lock();
    try
    {
        m_dequeue.push_back(c_t);
        m_waitCondition.wakeOne();
    }
    catch (...)
    {
        bret = false;//内存不足等
    }
    m_mtxQueue.unlock();
    return bret;
}

template<class T>
bool CGoerLockQueue<T>::push_front( const T &c_t )
{
    bool bret = true;
    m_mtxQueue.lock();
    try
    {
        m_dequeue.push_front(c_t);
        m_waitCondition.wakeOne();//设置唤醒阻塞
    }
    catch (...)
    {
        bret = false;//内存不足等
    }
    m_mtxQueue.unlock();
    return bret;
}

//有值返回true,没值返回false。
template<class T>
bool CGoerLockQueue<T>::get(GR_INT i, T &t)
{
    bool bret = true;
    m_mtxQueue.lock();

    if (m_dequeue.empty())
    {
        bret = false;
    }
    else
    {
        t = m_dequeue.at(i);
    }
    m_mtxQueue.unlock();
    return bret;
}

template<class T>
void CGoerLockQueue<T>::wait()
{
    if (m_dequeue.empty())
    {
        m_mtxWait.lock();
        m_waitCondition.wait(&m_mtxWait);//调用完成后会继续堵塞
        m_mtxWait.unlock();
    }
}

template<class T>
void CGoerLockQueue<T>::pop_front()
{
    m_mtxQueue.lock();
    m_dequeue.pop_front();
    m_mtxQueue.unlock();
}

template<class T>
void CGoerLockQueue<T>::pop_back()
{
    m_mtxQueue.lock();
    m_dequeue.pop_back();
    m_mtxQueue.unlock();
}


