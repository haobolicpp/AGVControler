/******************************************************************************
*
*文件名称：LockFreeRingBuffer.h
*摘    要：无锁环形缓冲区定义，仅适用于[单生产者]-[单消费者]
*
* ①缓冲区大小转为2的幂大小，方便将取余操作 x%isize 转换为 x&(isize-1)  可以自己试验一下
 * ②充分利用unsigned int溢出的特性，Write和Read是一直累加的，溢出后会回到0从头开始
 * ③Write==Read时，一定是为空了，缓冲区实际数据大小为Write-Read:
 * 说明一：特例：当W到最大时(2^32)，会回到头部0，如果R也在0，此时W==R，但队列不为空，但队列最大不可能到2^32，所以不可能发生
 * 情况二：Write溢出到头部，Read在后面（例如W==R==4294967291，写入1000个数据，W变为995，计算长度W-R=995-4294967291==1000）
*
*
******************************************************************************/
#ifndef LOCKFREERINGBUFFER_H
#define LOCKFREERINGBUFFER_H
#include <thread>
#include <atomic>
#include "commonlibrary_global.h"

class COMMONLIBRARYSHARED_EXPORT CLockFreeRingBuffer
{
public:
    //传入缓冲区最大容量，
    //注意，当满了的时候，内部不会自动扩展
    CLockFreeRingBuffer(unsigned int uiSize);
    ~CLockFreeRingBuffer();

    //初始化，false表示内存分配失败或超过最大容量大小
    bool Init();

    //put数据，返回false是空间不足，未添加成功
    bool PutData(const unsigned char *pBuffer, unsigned int uiLen);

    //get数据,返回实际取到的数据大小
    //pBuffer：需要在外面分配好
    //uiLen：希望返回的数据长度
    unsigned int GetData(unsigned char *pBuffer, unsigned int uiLen);

    /**
     * @brief PeekData 只查看数据，不会改变缓冲区内容
     * @param pBuffer 需要在外面分配好
     * @param uiLen 希望返回的数据长度
     * @return 返回实际读到的大小
     */
    unsigned int PeekData(unsigned char *pBuffer, unsigned int uiLen);

    //获取ringbuffer中数据长度的大小
    unsigned int DataLen();

    //清空环形缓冲区
    inline void Clean();

    //返回容量
    unsigned int GetCapacity(){return m_uiSize;};

    /*↓↓↓↓↓↓↓↓此区间接口为非线程安全的接口↓↓↓↓↓↓↓↓*/
    bool AdjustSize(unsigned int uiLen); //调整大小
    /*↑↑↑↑↑↑↑↑此区间接口为非线程安全的接口↑↑↑↑↑↑↑↑*/


private:
    //是否是2的幂
    inline bool is_power_of_2(unsigned int uiValue);
    //扩充指定的数到2的幂大小
    inline unsigned int roundup_power_of_2(unsigned int uiValue);
    //内存屏障
    void MemoryBarrierInternal();

private:
    //读下标
    unsigned int m_uiRead;
    //写下标
    unsigned int m_uiWrite;
    //缓冲区指针
    unsigned char *m_pRingBuffer;
    //缓冲区大小
    unsigned int m_uiSize;
    //上个线程ID
    std::atomic_int m_iLastThreadID;
};

#endif // LOCKFREERINGBUFFER_H
