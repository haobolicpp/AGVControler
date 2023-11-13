/******************************************************************************
*
*文件名称：QGoerTimeSpan.h
*摘    要：计算时间间隔
*
*
******************************************************************************/
#pragma once
#include <time.h>
#include <string>
#include "gr_type.h"
#include "commonlibrary_global.h"

using namespace std;

class COMMONLIBRARYSHARED_EXPORT CGoerTimeSpan
{
public:
    /******************************************************************************
    * 函数名称: CGoerTimeSpan
    * 功能描述: 构造函数
    * 输入参数: QString strFuncName:要打印时间的接口名称
    * GR_BOOL bAutoPrintTime:是否在析构函数中打印信息
    * GR_INT iOverTimePrint: 超过该时间(ms)，才打印数据
    * 输出参数: 无
    * 返 回 值: 无
    ******************************************************************************/
    CGoerTimeSpan(string strFuncName, GR_BOOL bAutoPrintTime=GR_TRUE, GR_INT iOverTimePrint=0);
    ~CGoerTimeSpan();

    //返回到调用该接口经过的时间
    GR_INT GetSpanTime();
    GR_U64 GetSpanTimeusec(); //微秒

    //立即输出,调用后析构不再打印
    void PrintNow();

private:
    //要打印的接口名称
    string m_strFuncName;
    //是否在析构函数中打印信息
    GR_BOOL m_bAutoPrintTime;
    //起始时间(ms)
    clock_t m_tStart;
    //微秒
    GR_U64 m_u64Start;
    //超过的时长ms
    GR_INT m_iOverTime;
};

