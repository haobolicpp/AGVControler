#include "CmdAddLine.h"
#include "MapView/MapGraphicsView.h"
#include "MapManager.h"

CCmdAddLine::CCmdAddLine(CMapGraphicsView *pView, EPathType eType, int iStartStation, int iEndStation)
{
    m_pView = pView;
    m_eType = eType;
    m_iStartStation = iStartStation;
    m_iEndStation = iEndStation;
    m_pLineItem = nullptr;
}

CCmdAddLine::~CCmdAddLine()
{

}

void CCmdAddLine::exec()
{
    TPath *path = CMapManager::GetInstance()->AddPath(m_eType, m_iStartStation, m_iEndStation);
    m_iPathID = path->iPathID;
    if (m_pLineItem == nullptr)
    {
        m_pLineItem = m_pView->AddLineItem(m_iPathID);
    }
    else
    {
        m_pView->scene()->addItem(m_pLineItem);
    }


}

void CCmdAddLine::undo()
{
    //清除Item，但不删除内存
    m_pView->scene()->removeItem(m_pLineItem);

    //删除内存
    CMapManager::GetInstance()->DeletePath(m_iPathID);
}

void CCmdAddLine::fullDelete()
{

}

void CCmdAddLine::insertDelete()
{
    CMapManager::GetInstance()->DeletePath(m_iPathID);
    if (m_pLineItem != nullptr)
    {
        delete m_pLineItem;
        m_pLineItem = nullptr;
    }
}
