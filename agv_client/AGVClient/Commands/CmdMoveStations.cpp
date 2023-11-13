#include "CmdMoveStations.h"
#include "MapManager.h"
#include "MapView/StationTextItem.h"
#include "MapView/MapGraphicsView.h"

CCmdMoveStations::CCmdMoveStations(std::list<TMoveStationsData> listStationItem)
{
    m_listMoveStations = listStationItem;
}

void CCmdMoveStations::exec()
{
    for (auto station : m_listMoveStations)
    {
        station.pItem->setPos(station.tNowStationPt);
        station.pItem->GetTextItem()->setPos(station.tNowStationTxtPt);

        auto itStation = CMapManager::GetInstance()->GetStation(station.pItem->GetStation());
        itStation->tPt = TPointd{station.tNowStationPt.x(), station.tNowStationPt.y()};

        //路线形状改变
        station.pItem->m_pView->StationMove(station.pItem->GetStation());
    }

}

void CCmdMoveStations::undo()
{
    for (auto station : m_listMoveStations)
    {
        station.pItem->setPos(station.tLastStationPt);
        station.pItem->GetTextItem()->setPos(station.tLastStationTxtPt);

        auto itStation = CMapManager::GetInstance()->GetStation(station.pItem->GetStation());
        itStation->tPt = TPointd{station.tLastStationPt.x(), station.tLastStationPt.y()};

        //路线形状改变
        station.pItem->m_pView->StationMove(station.pItem->GetStation());
    }

}

void CCmdMoveStations::fullDelete()
{

}

void CCmdMoveStations::insertDelete()
{

}
