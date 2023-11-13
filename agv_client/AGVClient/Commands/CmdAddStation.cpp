#include "CmdAddStation.h"
#include "MapManager.h"
#include "MapView/MapGraphicsView.h"
#include "QGraphicsScene"
#include "MapView/StationTextItem.h" //不包含该头文件，转成基类调用scene()会有问题！

CCmdAddStation::CCmdAddStation(CMapGraphicsView *pView, QPointF pt, double dAngle, EStationType type)
{
    m_pView = pView;
    m_dPt = pt;
    m_dAngle = dAngle;
    m_eType = type;
    m_pStationItem = nullptr;
}

CCmdAddStation::~CCmdAddStation()
{

}

void CCmdAddStation::exec()
{
    m_tStation = *CMapManager::GetInstance()->AddStation(m_eType, TPointd{m_dPt.x(), m_dPt.y()}, m_dAngle);
    if (m_pStationItem == nullptr)
    {
        m_pStationItem = m_pView->AddStationItem(&m_tStation);
    }
    else
    {
        m_pView->scene()->addItem(m_pStationItem);
        m_pView->scene()->addItem(m_pStationItem->GetTextItem());
    }
}

void CCmdAddStation::undo()
{
    //清除Item，但不删除内存
    m_pView->scene()->removeItem(m_pStationItem->GetTextItem());
    m_pView->scene()->removeItem(m_pStationItem);

    //删除内存
    CMapManager::GetInstance()->DeleteStation(m_tStation.iStationID);
}

void CCmdAddStation::fullDelete()
{

}

void CCmdAddStation::insertDelete()
{
    CMapManager::GetInstance()->DeleteStation(m_tStation.iStationID);
    if (m_pStationItem != nullptr)
    {
        delete m_pStationItem->GetTextItem();
        delete m_pStationItem;
    }
    m_pStationItem = nullptr;
}
