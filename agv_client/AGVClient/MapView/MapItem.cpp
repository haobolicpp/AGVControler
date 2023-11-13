#include <QPolygonF>
#include <QPen>
#include <QPainter>
#include <QStyleOptionGraphicsItem>
#include "AGVClient_def.h"
#include "MapItem.h"
#include "MapManager.h"
#include "GlogWrapper.h"
#include "MapView/MapGraphicsView.h"

CMapItem::CMapItem(CMapGraphicsView *pView, QGraphicsItem *parent):
    QGraphicsItem(parent)
{
    m_pPixmap = nullptr;

    QPolygonF polygonMapBouder;
    m_pView = pView;
    const TMapInfo &tmap = CMapManager::GetInstance()->GetCurrentMapInfo();

    int iWidth = tmap.iWidth/2 + 1;
    int iHeight = tmap.iHight/2 + 1;
    polygonMapBouder << QPointF(-iWidth, -iHeight) << QPointF(iWidth, -iHeight)
           << QPointF(iWidth,iHeight) << QPointF(-iWidth, iHeight)
           << QPointF(-iWidth, -iHeight);
    m_painterPath.addPolygon(polygonMapBouder);
}

QRectF CMapItem::boundingRect() const
{
    return m_painterPath.boundingRect();
}

void CMapItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    //画边框
    painter->setBrush(QColor(255, 255, 255)); //背景色
    painter->setPen(QPen(QColor(0, 255, 0), 1, Qt::PenStyle::DotLine)); //线条色
    painter->drawPath(m_painterPath); //绘制后会填充背景
    //画图像
    painter->drawPixmap(-m_pPixmap->width()/2.0-0.5, -m_pPixmap->height()/2.0-0.5, *m_pPixmap); //这个0.5的差值需要试验一下 TODO
}

void CMapItem::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
    emit m_pView->SignalMapPress();
    QGraphicsItem::mousePressEvent(event);
}

void CMapItem::UpdatePainterMap(bool bScanmap)
{
    clock_t ts = clock();
    TMapInfo tmapinfo;
    if (bScanmap)
    {
        tmapinfo = CMapManager::GetInstance()->GetScanMapInfo();
    }
    else
    {
        tmapinfo = CMapManager::GetInstance()->GetCurrentMapInfo();
    }

    //更新边界数据
    prepareGeometryChange();
    QPolygonF polygonMapBouder;

    QPainterPath pathTem;
    int iWidth = tmapinfo.iWidth/2 + 1;
    int iHeight = tmapinfo.iHight/2 + 1;
    polygonMapBouder << QPointF(-iWidth, -iHeight) << QPointF(iWidth, -iHeight)
           << QPointF(iWidth,iHeight) << QPointF(-iWidth, iHeight)
           << QPointF(-iWidth, -iHeight);
    pathTem.addPolygon(polygonMapBouder);
    m_painterPath = pathTem;

    //更新绘图内存
    if (m_pPixmap != nullptr)
    {
        delete m_pPixmap;
    }
    m_pPixmap = new QPixmap(tmapinfo.iWidth, tmapinfo.iHight);
    QPainter painter1(m_pPixmap);
    //绘制背景
    painter1.setBrush(QColor(255, 255, 255)); //背景色
    painter1.drawRect(QRect(0,0,tmapinfo.iWidth,tmapinfo.iHight));
    //绘制占用点
    painter1.setPen(QPen(QColor(0,0,0), 1));
    for (auto &it : tmapinfo.vecOccupy)
    {
        painter1.drawPoint(it.iX, it.iY);
    }
    //绘制未知点
    painter1.setPen(QPen(QColor(200, 200, 200), 1));
    for (auto &it : tmapinfo.vecUnknown)
    {
        painter1.drawPoint(it.iX, it.iY);
    }

    update();

    //GLOG_INFO << "UpdatePainterMap:" << clock()-ts << "ms";
}
