/******************************************************************************
*
*文件名称：LockFreeRingBuffer.cpp
*摘    要：无锁环形缓冲区实现，仅适用于[单生产者]-[单消费者]
*
*
******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <algorithm>
#include <string.h>
#include "pthread.h"
#include "LockFreeRingBuffer.h"
#include "gr_type.h"

#if defined(_WIN32) || defined(_WIN64)
#include <windows.h>
#endif

CLockFreeRingBuffer::CLockFreeRingBuffer(unsigned int uiSize)
{
    m_uiSize = uiSize;
    m_pRingBuffer = nullptr;
    m_uiRead = 0;
    m_uiWrite = 0;
    m_iLastThreadID = 0;
}

CLockFreeRingBuffer::~CLockFreeRingBuffer()
{
    if (m_pRingBuffer){
        free(m_pRingBuffer);
    }
}

bool CLockFreeRingBuffer::Init()
{
    return AdjustSize(m_uiSize);
}

bool CLockFreeRingBuffer::PutData(const unsigned char *pBuffer, unsigned int uiLen)
{
    assert(m_pRingBuffer != nullptr);

#if defined(Debug) || defined(QT_DEBUG)
    if (m_iLastThreadID.load() == 0){
        m_iLastThreadID.store(pthread_self());
    }else{
        if (pthread_self() != (unsigned long long)m_iLastThreadID){
            assert(false);
            return false;
        }
    }
#endif
    if (uiLen > (m_uiSize-DataLen()))
    {
        return false;
    }

    //添加内存屏障
    MemoryBarrierInternal();

    //计算写下标在uiSize内的实际位置(取余)
    unsigned int uiWriteTrue = m_uiWrite & (m_uiSize -1);

    //计算可写空间（技巧）
    unsigned int l = MIN(uiLen, m_uiSize - uiWriteTrue);

    /* first put the data starting from fifo->in to buffer end */
    //拷贝到尾部
    memcpy(m_pRingBuffer+uiWriteTrue, pBuffer, l);

    /* then put the rest (if any) at the beginning of the buffer */
    //拷贝到头部(如果上一个步骤都拷贝完了，此时uilen-l==0，此步骤什么都没干)
    memcpy(m_pRingBuffer, pBuffer+l, uiLen-l);

    //添加内存屏障
    MemoryBarrierInternal();

    m_uiWrite += uiLen;

    return true;

}

unsigned int CLockFreeRingBuffer::GetData(unsigned char *pBuffer, unsigned int uiLen)
{
    unsigned int uiCanRead;

    uiCanRead = PeekData(pBuffer, uiLen);

    m_uiRead += uiCanRead;

    return uiCanRead;

}

unsigned int CLockFreeRingBuffer::PeekData(unsigned char *pBuffer, unsigned int uiLen)
{
    //实际可读空间大小
    unsigned int uiCanRead = MIN(uiLen, DataLen());

    //添加内存屏障
    MemoryBarrierInternal();

    //计算读下标的实际位置(取余)
    unsigned int uiReadTrue = m_uiRead & (m_uiSize-1);

    //（技巧）
    unsigned int l = MIN(uiCanRead, m_uiSize - uiReadTrue);
    //尾部取出
    memcpy(pBuffer, m_pRingBuffer+uiReadTrue, l);

    //头部取出
    memcpy(pBuffer+l, m_pRingBuffer, uiCanRead-l);

    //添加内存屏障
    MemoryBarrierInternal();

    return uiCanRead;
}

unsigned int CLockFreeRingBuffer::DataLen()
{
    return m_uiWrite - m_uiRead;
}

void CLockFreeRingBuffer::Clean()
{
    m_uiRead = 0;
    m_uiWrite = 0;;
}

bool CLockFreeRingBuffer::AdjustSize(unsigned int uiLen)
{
    //大小与实际数据比较
    if (DataLen() > uiLen){
        return false;
    }

    unsigned int uiTempSize = DataLen();

    //如果不是2的幂，那么转换之前要先判断该数值不能>0x80000000,
    //因为转换2的幂函数要求不能超过该值，否则while会有问题
    if (!this->is_power_of_2(uiLen))
    {
        if (uiLen > (unsigned int)0x80000000)
        {
            return false;
        }
        uiLen = roundup_power_of_2(uiLen);//转换为2的幂
    }
    unsigned char *pTempbuffer = (unsigned char *)malloc(uiLen);
    if (nullptr == pTempbuffer)
    {
        return false;
    }
    if (m_pRingBuffer == NULL){
        m_pRingBuffer = pTempbuffer;
    }else{
        PeekData(pTempbuffer, uiTempSize);//拷贝有效数据
        free(m_pRingBuffer);
        m_pRingBuffer = pTempbuffer;
        m_uiRead = 0;
        m_uiWrite = uiTempSize;
    }
    m_uiSize = uiLen;

    return true;
}

//n & (n-1),转换为二进制可以试验一下。
bool CLockFreeRingBuffer::is_power_of_2(unsigned int uiValue)
{
    return (uiValue !=0 && ((uiValue & (uiValue-1))==0));
}

unsigned int CLockFreeRingBuffer::roundup_power_of_2(unsigned int uiValue)
{
    if (is_power_of_2(uiValue))
    {
        return uiValue;
    }

    unsigned int uiandv = (unsigned int)(~(unsigned int) 0);//结果为1111 1111 1111 1111
    uiandv = ~(uiandv>>1);//结果为1000 0000 0000 0000

    //如5往上的幂为8，5(0101),8(1000),即将0101的上一位为1的二进制即可
    //算法核心思路为从最大1000 0000 0000 0000(后面省略了) 开始，每次右移一位进行比较
    //下一次比较的即为0100 0000 0000 0000
    while ((uiandv & uiValue) == 0)
    {
        uiandv = uiandv >> 1;
    }

    //最终如5，在uiandv==0100时跳出，此时再左移一位即可(1000)
    return uiandv<<1;
}

void CLockFreeRingBuffer::MemoryBarrierInternal()
{
#if defined WIN32 || defined _WIN64
    MemoryBarrier();
#else
    __sync_synchronize();
#endif
}
