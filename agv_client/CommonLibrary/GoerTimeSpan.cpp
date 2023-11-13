/******************************************************************************
*
*文件名称：QGoerTimeSpan.cpp
*摘    要：计算时间间隔
*
******************************************************************************/
#ifndef _WIN32
#include <sys/time.h>
#else
#endif
#include <QDebug>
#include "GoerTimeSpan.h"
#include "GoerToolUtile.h"
#include "GlogWrapper.h"

#define BUFFER_SIZE 1024

CGoerTimeSpan::CGoerTimeSpan(string strFuncName, GR_BOOL bAutoPrintTime, GR_INT iOverTimePrint)
{
    m_strFuncName = strFuncName;
    m_bAutoPrintTime = bAutoPrintTime;
    m_tStart = CGoerToolUtile::GetCurrentTimeMSec();
    m_iOverTime = iOverTimePrint;
#ifndef _WIN32
    struct timeval tv;
    gettimeofday(&tv, NULL);
    m_u64Start = tv.tv_sec*1000000 + tv.tv_usec;
#endif
}

CGoerTimeSpan::~CGoerTimeSpan()
{
    if (m_bAutoPrintTime)
    {
        PrintNow();
    }
}

GR_INT CGoerTimeSpan::GetSpanTime()
{
    return CGoerToolUtile::GetCurrentTimeMSec() - m_tStart;
}

GR_U64 CGoerTimeSpan::GetSpanTimeusec()
{
#ifndef _WIN32
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec*1000000 + tv.tv_usec - m_u64Start;
#else
    return 0;
#endif
}

void CGoerTimeSpan::PrintNow()
{
    GR_CHAR chBuffer[BUFFER_SIZE] = {0};
    GR_INT ispan = (int)(CGoerToolUtile::GetCurrentTimeMSec() - m_tStart);
    if (ispan > m_iOverTime)
    {
        snprintf(chBuffer, BUFFER_SIZE, "%s-超过%dms: %dms", m_strFuncName.c_str(), m_iOverTime, ispan);
        //printf("%s", chBuffer);
        //LOG_INFO("%s", chBuffer);
        GLOG_INFO << chBuffer;
    }
    m_bAutoPrintTime = GR_FALSE;
}
