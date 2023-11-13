#include "CmdModifyStation.h"
#include "MapView/StationTextItem.h"
#include <QGraphicsScene>
#include "MapManager.h"
#include "MapView/MapGraphicsView.h"

CCmdModifyStation::CCmdModifyStation(CStationItem *pItem, QPointF tLastPt, double dLastAngle,
                                     QPointF tNowPt, double dNowAngle, QPointF lastTxtPt, QPointF nowTxtPt)
{
    m_pStationItem = pItem;
    m_dLasttPt = tLastPt;
    m_lastdAngle = dLastAngle;
    m_dNowtPt = tNowPt;
    m_nowdAngle = dNowAngle;
    m_tlastTxt = lastTxtPt;
    m_tnowTxt = nowTxtPt;
}

void CCmdModifyStation::exec()
{
    QTransform transStation = m_pStationItem->transform();

    //平移
    auto itStation = CMapManager::GetInstance()->GetStation(m_pStationItem->GetStation());
    m_pStationItem->setPos(m_dNowtPt);
    itStation->tPt = TPointd{m_dNowtPt.x(), m_dNowtPt.y()};
    m_pStationItem->GetTextItem()->setPos(m_tnowTxt);
    //旋转
    double degdiff = m_nowdAngle - m_lastdAngle;
    transStation.rotate(degdiff); //顺时针旋转坐标系，因为y轴向下，所以顺时针旋转
    itStation->dAngle = m_nowdAngle;

    m_pStationItem->setTransform(transStation);

    //通知线段站点位置变动了
    m_pStationItem->m_pView->StationMove(m_pStationItem->GetStation());

}

void CCmdModifyStation::undo()
{
    QTransform transStation = m_pStationItem->transform();
    auto itStation = CMapManager::GetInstance()->GetStation(m_pStationItem->GetStation());

    //平移
    m_pStationItem->setPos(m_dLasttPt);
    itStation->tPt = TPointd{m_dLasttPt.x(), m_dLasttPt.y()};
    m_pStationItem->GetTextItem()->setPos(m_tlastTxt);
    //旋转
    double degdiff = m_lastdAngle - m_nowdAngle;
    transStation.rotate(degdiff); //顺时针旋转坐标系，因为y轴向下，所以顺时针旋转

    m_pStationItem->setTransform(transStation);

    //通知线段站点位置变动了
    m_pStationItem->m_pView->StationMove(m_pStationItem->GetStation());

}

void CCmdModifyStation::fullDelete()
{

}

void CCmdModifyStation::insertDelete()
{

}
