/******************************************************************************
*
*文件名称：AutoIncBuffer.h
*摘    要：自增长缓冲区类声明。
*
*
******************************************************************************/
#ifndef _AUTOINCBUFFER_H
#define _AUTOINCBUFFER_H
#include "gr_type.h"
#include "commonlibrary_global.h"

class COMMONLIBRARYSHARED_EXPORT CAutoIncBuffer
{
public:
    CAutoIncBuffer();
    ~CAutoIncBuffer();

    /******************************************************************************
    * 函数名称: Init
    * 功能描述: 初始化模块
    * 输入参数: GR_INT iSize：初始化大小
    * 输出参数: 无
    * 返 回 值: GR_TRUE成功 ，GR_FALSE失败
    ******************************************************************************/
    GR_BOOL Init(GR_INT iSize);

    /******************************************************************************
    * 函数名称: AdjustSize
    * 功能描述: 调整大小
    * 输入参数: GR_INT iNewSize：调整后的大小
    * 输出参数: 无
    * 返 回 值: GR_TRUE成功 ,GR_FALSE表示内存调整失败
    ******************************************************************************/
    GR_BOOL AdjustSize(GR_INT iNewSize);

    //返回buffer指针
    GR_PCHAR GetBuffer();

private:
    //数据指针
    GR_PCHAR m_pBuffer;
    //长度
    GR_INT m_iLen;
};

#endif
