/******************************************************************************
*文件名称：AutoIncBuffer.cpp
*摘    要：自增长缓冲区类实现。
*
******************************************************************************/
#include <stdlib.h>
#include "AutoIncBuffer.h"

CAutoIncBuffer::CAutoIncBuffer()
{
    m_pBuffer = GR_NULL;
    m_iLen = 0;
}

CAutoIncBuffer::~CAutoIncBuffer()
{
//    if (m_pBuffer != GR_NULL)
//    {
//        free(m_pBuffer);
//        m_pBuffer = GR_NULL;
//    }
}

GR_BOOL CAutoIncBuffer::Init(GR_INT iSize)
{
    m_pBuffer = (GR_PCHAR)malloc(iSize);
    if (GR_NULL ==  m_pBuffer)
    {
        return GR_FALSE;
    }
    m_iLen = iSize;
    return GR_TRUE;
}

GR_BOOL CAutoIncBuffer::AdjustSize(GR_INT iNewSize)
{
    if (iNewSize <= m_iLen)
    {
        return GR_TRUE;
    }
    if (0 == m_iLen)
    {
        return Init(iNewSize);
    }
    GR_PCHAR pBufferTem = (GR_PCHAR)realloc(m_pBuffer, iNewSize);
    if (pBufferTem != GR_NULL)
    {
        m_pBuffer = pBufferTem;
        m_iLen = iNewSize;
        return GR_TRUE;
    }

    return GR_FALSE;
}

GR_PCHAR CAutoIncBuffer::GetBuffer()
{
    return m_pBuffer;
}
