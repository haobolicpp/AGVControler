#include "CmdDelete.h"
#include "MapView/StationTextItem.h"
#include "MapManager.h"

CCmdDelete::CCmdDelete(CMapGraphicsView *pView, std::set<CStationItem*> &setStaion, std::set<CLineItem*> &setLine)
{
    m_pView = pView;
    for (auto it : setStaion)
    {
        TStation *pStation = CMapManager::GetInstance()->GetStation(it->GetStation());
        m_listStaion.push_back(TStationData{it, *pStation});
    }
    for (auto it : setLine)
    {
        TPath *pPath = CMapManager::GetInstance()->GetPath(it->GetPath());
        m_listLine.push_back(TLineData{it, *pPath});
    }
}

void CCmdDelete::exec()
{
    for (auto &line : m_listLine)
    {
        //删除Item，只从场景中删除
        m_pView->scene()->removeItem(line.pItem);
         //删除内存数据
        CMapManager::GetInstance()->DeletePath(line.tPath.iPathID);
    }
    for (auto &station : m_listStaion)
    {
        m_pView->scene()->removeItem(station.pItem->GetTextItem());
        m_pView->scene()->removeItem(station.pItem);
        CMapManager::GetInstance()->DeleteStation(station.tStation.iStationID);
    }
}

void CCmdDelete::undo()
{
    for (auto &data : m_listLine)
    {
        m_pView->scene()->addItem(data.pItem);
        CMapManager::GetInstance()->AddPathExisting(data.tPath);
    }
    for (auto &data : m_listStaion)
    {
        m_pView->scene()->addItem(data.pItem->GetTextItem());
        m_pView->scene()->addItem(data.pItem);
        CMapManager::GetInstance()->AddStationExisting(data.tStation);
    }
}

void CCmdDelete::fullDelete()
{
    for (auto &data : m_listLine)
    {
        delete data.pItem;
    }
    for (auto &data : m_listStaion)
    {
        delete data.pItem->GetTextItem();
        delete data.pItem;
    }

    m_listLine.clear();
    m_listStaion.clear();
}

void CCmdDelete::insertDelete()
{

}
