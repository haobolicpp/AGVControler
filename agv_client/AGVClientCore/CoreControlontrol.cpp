#include "CoreControl.h"

SINGLETON_IMPLEMENT(CCoreControl)

CCoreControl::CCoreControl()
{
    m_bCurrentMapModify = false;
    m_agvStatus = EAGVRunStatus::DoNothing;
}

void CCoreControl::StartScanMap(bool bScanMap)
{
    if (bScanMap)
    {
        m_agvStatus = EAGVRunStatus::Scan_Start;
    }
    else
    {
        m_agvStatus = EAGVRunStatus::DoNothing;
    }
}

bool CCoreControl::IsScanMap()
{
    if (m_agvStatus == EAGVRunStatus::Scan_Start || m_agvStatus == EAGVRunStatus::Scan_RecvMap)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void CCoreControl::UpdateStatusByRecvScanMap()
{
    m_agvStatus = EAGVRunStatus::Scan_RecvMap;
}
