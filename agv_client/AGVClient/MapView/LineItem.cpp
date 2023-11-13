/*
 * LineItem中心点和scene是重合的，本Item的各种pos()和scene中一致
*/
#include <QPainter>
#include <QStyleOptionGraphicsItem>
#include <QPolygonF>
#include <QDebug>
#include <QGraphicsSceneHoverEvent>
#include <QPainterPathStroker>
#include <list>
#include <math.h>

#include "LineItem.h"
#include "MapView_def.h"
#include "MapGraphicsView.h"
#include "GeometryAlgorithm.h"
#include "MapManager.h"

CLineItem::CLineItem(int iPahtID, CMapGraphicsView *pView, QGraphicsItem *parent):
    QGraphicsItem(parent)
{
    m_pView = pView;
    m_bIsHoverEnter = false;
    m_bIsPressCtrlPoint1 = false;
    m_bIsPressCtrlPoint2 = false;
    m_bIsDrawCtrlLine = false;
    m_iPathID = iPahtID;
    m_allPath.setFillRule(Qt::WindingFill);
    auto itPath = CMapManager::GetInstance()->GetPath(m_iPathID);
    auto itStartStation = CMapManager::GetInstance()->GetStation(itPath->iStartStation);
    auto itEndStation = CMapManager::GetInstance()->GetStation(itPath->iEndStation);
    if(EPathType::PathType_Line == itPath->eType)
    {
        //直线
        TPointd tTempPt;
        tTempPt.dX = (2 * itStartStation->tPt.dX + itEndStation->tPt.dX) / 3;
        tTempPt.dY = (2 * itStartStation->tPt.dY + itEndStation->tPt.dY) / 3;
        TPointd tEndPt1 = CGeometryAlgorithm::CalcEndPointSeg(tTempPt, itStartStation->tPt);
        tTempPt.dX = (itStartStation->tPt.dX + 2 * itEndStation->tPt.dX) / 3;
        tTempPt.dY = (itStartStation->tPt.dY + 2 * itEndStation->tPt.dY) / 3;
        TPointd tEndPt2 = CGeometryAlgorithm::CalcEndPointSeg(tTempPt, itEndStation->tPt);
        itPath->tStartToEndPt = tEndPt1;
        itPath->tEndToEndPt = tEndPt2;
    }
    else if(EPathType::PathType_Circle == itPath->eType)
    {
        //圆
    }
    else
    {
        //贝塞尔曲线

        TPointd tEndPt1 = CGeometryAlgorithm::CalcEndPointSeg(itPath->udata.tBazierData.tPtNearStart, itStartStation->tPt);
        TPointd tEndPt2 = CGeometryAlgorithm::CalcEndPointSeg(itPath->udata.tBazierData.tPtNearEnd, itEndStation->tPt);
        itPath->tStartToEndPt = tEndPt1;
        itPath->tEndToEndPt = tEndPt2;
    }
    prepareGeometryChange();
    QPointF qStartStation, qEndStation;
    qStartStation.setX(itStartStation->tPt.dX);
    qStartStation.setY(itStartStation->tPt.dY);
    qEndStation.setX(itEndStation->tPt.dX);
    qEndStation.setY(itEndStation->tPt.dY);
    if(EPathType::PathType_Bezier == itPath->eType)  //贝塞尔
    {
        //控制点
        QPointF qCrtPointFirst, qCrtPointSecond;
        qCrtPointFirst.setX(itPath->udata.tBazierData.tPtNearStart.dX);
        qCrtPointFirst.setY(itPath->udata.tBazierData.tPtNearStart.dY);
        m_CrtlPointFirst.addEllipse(qCrtPointFirst, LINE_CTRLPOINT_LEN, LINE_CTRLPOINT_LEN);

        qCrtPointSecond.setX(itPath->udata.tBazierData.tPtNearEnd.dX);
        qCrtPointSecond.setY(itPath->udata.tBazierData.tPtNearEnd.dY);
        m_CrtlPointSecond.addEllipse(qCrtPointSecond, LINE_CTRLPOINT_LEN, LINE_CTRLPOINT_LEN);

        qStartStation.setX(itStartStation->tPt.dX);
        qStartStation.setY(itStartStation->tPt.dY);
        qEndStation.setX(itEndStation->tPt.dX);
        qEndStation.setY(itEndStation->tPt.dY);
        m_Line.moveTo(qStartStation);
        m_Line.cubicTo(qCrtPointFirst, qCrtPointSecond, qEndStation);
        QPainterPathStroker qLineStroker;
        qLineStroker.setWidth(LINE_CTRLPOINT_LEN-2);
        m_Line = qLineStroker.createStroke(m_Line);

        m_allPath.addPath(m_CrtlPointFirst);
        m_allPath.addPath(m_CrtlPointSecond);
        m_allPath.addPath(m_Line);
    }
    else if(EPathType::PathType_Line == itPath->eType) //直线
    {
        QPainterPathStroker qLineStroker;
        m_Line.moveTo(qStartStation);
        m_Line.lineTo(qEndStation);
        qLineStroker.setWidth(LINE_CTRLPOINT_LEN-2);
        m_Line = qLineStroker.createStroke(m_Line);
        m_allPath.addPath(m_Line);
    }
    else  //圆形
    {
        assert("error");
    }

    //标记
    setFlags(QGraphicsItem::ItemIsSelectable /*| QGraphicsItem::ItemIsMovable*/);
    setAcceptHoverEvents(true);//可激发hoverXXXEvent回调
}

void CLineItem::MouseMove(QPointF pos)
{
    pos = mapFromScene(pos);

    if (isSelected())
    {
        if (m_CrtlPointFirst.contains(pos)|| m_CrtlPointSecond.contains(pos))
        {
            setCursor(QCursor(Qt::CursorShape::CrossCursor));
            m_bIsHoverEnter = true;
        }else if (m_Line.contains(pos)){
            setCursor(QCursor(Qt::CursorShape::SizeAllCursor));
            m_bIsHoverEnter = true;
        }
        else
        {
            setCursor(QCursor(Qt::CursorShape::ArrowCursor));
            m_bIsHoverEnter = false;
        }
    }
    else
    {
        if (m_Line.contains(pos))
        {
            //setCursor(QCursor(Qt::CursorShape::CrossCursor));
            m_bIsHoverEnter = true;
        }else{
            //setCursor(QCursor(Qt::CursorShape::ArrowCursor));
            m_bIsHoverEnter = false;
        }
    }
    update();
}

void CLineItem::StationMove()
{
    prepareGeometryChange();
    auto itPath = CMapManager::GetInstance()->GetPath(m_iPathID);
    auto itStartStation = CMapManager::GetInstance()->GetStation(itPath->iStartStation);
    auto itEndStation = CMapManager::GetInstance()->GetStation(itPath->iEndStation);
    if(EPathType::PathType_Line == itPath->eType)
    {
        //直线
        QPointF qStartStation, qEndStation;
        QPainterPath temp,temp4;
        qStartStation.setX(itStartStation->tPt.dX);
        qStartStation.setY(itStartStation->tPt.dY);
        qEndStation.setX(itEndStation->tPt.dX);
        qEndStation.setY(itEndStation->tPt.dY);
        temp.moveTo(qStartStation);
        temp.lineTo(qEndStation);
        m_Line = temp;
        QPainterPathStroker qLineStroker, qLineTemp;
        qLineStroker.setWidth(LINE_CTRLPOINT_LEN-2);
        temp = qLineTemp.createStroke(temp);
        m_Line = qLineStroker.createStroke(m_Line);
        temp4.addPath(m_Line);
        m_allPath = temp4;
        m_allPath.setFillRule(Qt::WindingFill);
    }
    else if(EPathType::PathType_Circle == itPath->eType)
    {
        //圆
    }
    else
    {
        //贝塞尔曲线
        QPointF qCrtPointFirst, qCrtPointSecond, qStartStation, qEndStation;
        QPainterPath temp,temp4;

        qStartStation.setX(itStartStation->tPt.dX);
        qStartStation.setY(itStartStation->tPt.dY);
        qEndStation.setX(itEndStation->tPt.dX);
        qEndStation.setY(itEndStation->tPt.dY);
        qCrtPointFirst.setX(itPath->udata.tBazierData.tPtNearStart.dX);
        qCrtPointFirst.setY(itPath->udata.tBazierData.tPtNearStart.dY);
        qCrtPointSecond.setX(itPath->udata.tBazierData.tPtNearEnd.dX);
        qCrtPointSecond.setY(itPath->udata.tBazierData.tPtNearEnd.dY);
        temp.moveTo(qStartStation);
        temp.cubicTo(qCrtPointFirst, qCrtPointSecond, qEndStation);
        m_Line = temp;
        QPainterPathStroker qLineStroker, qLineTemp;
        qLineStroker.setWidth(LINE_CTRLPOINT_LEN-2);
        temp = qLineTemp.createStroke(temp);
        m_Line = qLineStroker.createStroke(m_Line);
        temp4.addPath(m_CrtlPointFirst);
        temp4.addPath(m_CrtlPointSecond);
        temp4.addPath(m_Line);
        m_allPath = temp4;
        m_allPath.setFillRule(Qt::WindingFill);
    }
}

QRectF CLineItem::boundingRect() const
{
    return m_allPath.boundingRect();
}

QPainterPath CLineItem::shape() const
{
    return m_allPath;
}

void CLineItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    painter->setRenderHints(QPainter::Antialiasing);//抗锯齿
    double dOpacity = 0.5;//不透明度
    painter->setOpacity(dOpacity);
    int ipenWidth = LINE_WIDTH_NO_SELECTED; //画笔宽度
    if(isSelected())
    {
        ipenWidth = LINE_WIDTH_SELECTED_HOVER;
    }
    else
    {
        m_bIsDrawCtrlLine = false;
    }
    if(m_bIsHoverEnter)
    {
        ipenWidth = LINE_WIDTH_SELECTED_HOVER;
    }
    else
    {
        ipenWidth = LINE_WIDTH_NO_SELECTED;
    }
    //根据线类型进行绘画
    auto itPath = CMapManager::GetInstance()->GetPath(m_iPathID);
    if (itPath==nullptr)
    {
        return;
    }
    auto itStartStation = CMapManager::GetInstance()->GetStation(itPath->iStartStation);
    if (itStartStation==nullptr)
    {
        return;
    }
    auto itEndStation = CMapManager::GetInstance()->GetStation(itPath->iEndStation);
    if (itEndStation==nullptr)
    {
        return;
    }
    if(EPathType::PathType_Line == itPath->eType)
    {
        //直线
        painter->setPen(QPen(QColor(0, 255, 0), ipenWidth, Qt::PenStyle::SolidLine));
        painter->setBrush(QColor(0, 255, 0));
        QPointF qStartStation, qEndStation;
        QPainterPath temp,temp4;
        qStartStation.setX(itStartStation->tPt.dX);
        qStartStation.setY(itStartStation->tPt.dY);
        qEndStation.setX(itEndStation->tPt.dX);
        qEndStation.setY(itEndStation->tPt.dY);
        temp.moveTo(qStartStation);
        temp.lineTo(qEndStation);
        m_Line = temp;
        QPainterPathStroker qLineStroker, qLineTemp;
        qLineStroker.setWidth(LINE_CTRLPOINT_LEN-2);
        temp = qLineTemp.createStroke(temp);
        m_Line = qLineStroker.createStroke(m_Line);
        temp4.addPath(m_Line);
        m_allPath = temp4;
        m_allPath.setFillRule(Qt::WindingFill);
        if(LINE_WIDTH_NO_SELECTED == ipenWidth)
        {
            painter->drawPath(temp);
        }
        else
        {
            painter->drawPath(m_Line);
        }

        //绘制延长线
        painter->setPen(QPen(QColor(238,130,238), 1, Qt::PenStyle::DotLine));
        TPointd tTempPt;
        tTempPt.dX = (2 * itStartStation->tPt.dX + itEndStation->tPt.dX) / 3;
        tTempPt.dY = (2 * itStartStation->tPt.dY + itEndStation->tPt.dY) / 3;
        TPointd tEndPt1 = CGeometryAlgorithm::CalcEndPointSeg(tTempPt, itStartStation->tPt);
        tTempPt.dX = (itStartStation->tPt.dX + 2 * itEndStation->tPt.dX) / 3;
        tTempPt.dY = (itStartStation->tPt.dY + 2 * itEndStation->tPt.dY) / 3;
        TPointd tEndPt2 = CGeometryAlgorithm::CalcEndPointSeg(tTempPt, itEndStation->tPt);
        itPath->tStartToEndPt = tEndPt1;
        itPath->tEndToEndPt = tEndPt2;
        painter->drawLine(itStartStation->tPt.dX, itStartStation->tPt.dY,
                          itPath->tStartToEndPt.dX, itPath->tStartToEndPt.dY);
        painter->drawLine(itEndStation->tPt.dX, itEndStation->tPt.dY,
                          itPath->tEndToEndPt.dX, itPath->tEndToEndPt.dY);

        //绘制箭头
        TPointd tMid, tMidToStart, tMidToEnd;
        TPointd tPt1, tPt2;
        painter->setBrush(QColor(255, 100, 0));
        painter->setPen(QPen(QColor(0,255, 0), 1, Qt::PenStyle::SolidLine));
        //计算箭头端点
        CGeometryAlgorithm::CalcPointByThreePointAndLength(itStartStation->tPt, itEndStation->tPt,
                                    LINE_ARROW_WIDTH, tMidToStart, tMidToEnd, tMid);
        if (itPath->eDirect == EPathDirect::PathDirect_Bothway || itPath->eDirect == EPathDirect::PathDirect_EndToStart)
        {
            //计算箭头边点
            if (CGeometryAlgorithm::CalcPointByRightTriangleTwoPtAndOneSideLength(tMid, tMidToStart, LINE_ARROW_WIDTH, tPt1, tPt2))
            {
                QPolygonF polygon;
                QPainterPath pathArrow;
                polygon << QPointF(tMidToStart.dX, tMidToStart.dY) << QPointF(tPt1.dX,tPt1.dY) << QPointF(tPt2.dX,tPt2.dY) << QPointF(tMidToStart.dX, tMidToStart.dY);
                pathArrow.addPolygon(polygon);
                painter->drawPath(pathArrow);
            }

        }
        if (itPath->eDirect == EPathDirect::PathDirect_Bothway || itPath->eDirect == EPathDirect::PathDirect_StartToEnd)
        {
            if (CGeometryAlgorithm::CalcPointByRightTriangleTwoPtAndOneSideLength(tMid, tMidToEnd, LINE_ARROW_WIDTH, tPt1, tPt2))
            {
                //绘制到终点的箭头
                QPolygonF polygon;
                QPainterPath pathArrow;
                polygon << QPointF(tMidToEnd.dX, tMidToEnd.dY) << QPointF(tPt1.dX,tPt1.dY) << QPointF(tPt2.dX,tPt2.dY) << QPointF(tMidToEnd.dX, tMidToEnd.dY);
                pathArrow.addPolygon(polygon);
                painter->drawPath(pathArrow);
            }
        }
        else{}

    }
    else if(EPathType::PathType_Circle == itPath->eType)
    {
        //圆
        painter->setPen(QPen(QColor(255, 0, 255), ipenWidth, Qt::PenStyle::SolidLine));
    }
    else
    {
        QPointF qCrtPointFirst, qCrtPointSecond, qStartStation, qEndStation;
        QPainterPath temp,temp4;
        qStartStation.setX(itStartStation->tPt.dX);
        qStartStation.setY(itStartStation->tPt.dY);
        qEndStation.setX(itEndStation->tPt.dX);
        qEndStation.setY(itEndStation->tPt.dY);
        qCrtPointFirst.setX(itPath->udata.tBazierData.tPtNearStart.dX);
        qCrtPointFirst.setY(itPath->udata.tBazierData.tPtNearStart.dY);
        qCrtPointSecond.setX(itPath->udata.tBazierData.tPtNearEnd.dX);
        qCrtPointSecond.setY(itPath->udata.tBazierData.tPtNearEnd.dY);
        temp.moveTo(qStartStation);
        temp.cubicTo(qCrtPointFirst, qCrtPointSecond, qEndStation);
        m_Line = temp;
        QPainterPathStroker qLineStroker, qLineTemp;
        qLineStroker.setWidth(LINE_CTRLPOINT_LEN-2);
        temp = qLineTemp.createStroke(temp);
        m_Line = qLineStroker.createStroke(m_Line);
        temp4.addPath(m_CrtlPointFirst);
        temp4.addPath(m_CrtlPointSecond);
        temp4.addPath(m_Line);
        m_allPath = temp4;
        m_allPath.setFillRule(Qt::WindingFill);
        //三阶贝塞尔曲线
        painter->setBrush(QColor(184, 134, 11));
        painter->setPen(QPen(QColor(184, 134, 11), ipenWidth, Qt::PenStyle::SolidLine));
        if(LINE_WIDTH_NO_SELECTED == ipenWidth)
        {
            painter->drawPath(temp);
        }
        else
        {
            painter->drawPath(m_Line);
        }

        painter->setPen(QPen(QColor(238,130,238), 1, Qt::PenStyle::DotLine));

        //绘制延长线
        TPointd tEndPt1 = CGeometryAlgorithm::CalcEndPointSeg(itPath->udata.tBazierData.tPtNearStart, itStartStation->tPt);
        TPointd tEndPt2 = CGeometryAlgorithm::CalcEndPointSeg(itPath->udata.tBazierData.tPtNearEnd, itEndStation->tPt);
        itPath->tStartToEndPt = tEndPt1;
        itPath->tEndToEndPt = tEndPt2;
        painter->drawLine(itStartStation->tPt.dX, itStartStation->tPt.dY,
                          itPath->tStartToEndPt.dX, itPath->tStartToEndPt.dY);
        painter->drawLine(itEndStation->tPt.dX, itEndStation->tPt.dY,
                          itPath->tEndToEndPt.dX, itPath->tEndToEndPt.dY);

        //绘制箭头
        painter->setBrush(QColor(255, 100, 0));
        painter->setPen(QPen(QColor(0,255, 0), 1, Qt::PenStyle::SolidLine));
        QPointF tMid, tMidToStart, tMidToEnd;
        CalcBezierMidPoint(itStartStation->tPt, itPath->udata.tBazierData.tPtNearStart, itPath->udata.tBazierData.tPtNearEnd, itEndStation->tPt,
                           tMid, tMidToStart, tMidToEnd);
        TPointd tPt1,tPt2;
        if (itPath->eDirect == EPathDirect::PathDirect_Bothway || itPath->eDirect == EPathDirect::PathDirect_EndToStart)
        {
            //绘制到起点的箭头
            if (CGeometryAlgorithm::CalcPointByRightTriangleTwoPtAndOneSideLength(TPointd{tMid.x(),tMid.y()}, TPointd{tMidToStart.x(),tMidToStart.y()},
                                                                                  LINE_ARROW_WIDTH, tPt1, tPt2))
            {
                QPolygonF polygon;
                QPainterPath pathArrow;
                polygon << tMidToStart << QPointF(tPt1.dX,tPt1.dY) << QPointF(tPt2.dX,tPt2.dY) << tMidToStart;
                pathArrow.addPolygon(polygon);
                painter->drawPath(pathArrow);
            }
        }
        if (itPath->eDirect == EPathDirect::PathDirect_Bothway || itPath->eDirect == EPathDirect::PathDirect_StartToEnd)
        {
            if (CGeometryAlgorithm::CalcPointByRightTriangleTwoPtAndOneSideLength(TPointd{tMid.x(),tMid.y()}, TPointd{tMidToEnd.x(),tMidToEnd.y()},
                                                                                  LINE_ARROW_WIDTH, tPt1, tPt2))
            {
                //绘制到终点的箭头
                QPolygonF polygon;
                QPainterPath pathArrow;
                polygon << tMidToEnd << QPointF(tPt1.dX,tPt1.dY) << QPointF(tPt2.dX,tPt2.dY) << tMidToEnd;
                pathArrow.addPolygon(polygon);
                painter->drawPath(pathArrow);
            }
        }
        else{}
    }
    if(m_bIsDrawCtrlLine)
    {
        if(itPath->eType != EPathType::PathType_Bezier )
        {
            return;
        }
        painter->setPen(QPen(QColor(0, 255, 0), 1, Qt::PenStyle::DotLine));
        painter->drawLine(itStartStation->tPt.dX, itStartStation->tPt.dY,
                          itPath->udata.tBazierData.tPtNearStart.dX, itPath->udata.tBazierData.tPtNearStart.dY);
        painter->drawLine(itPath->udata.tBazierData.tPtNearStart.dX, itPath->udata.tBazierData.tPtNearStart.dY,
                          itPath->udata.tBazierData.tPtNearEnd.dX, itPath->udata.tBazierData.tPtNearEnd.dY);
        painter->drawLine(itPath->udata.tBazierData.tPtNearEnd.dX, itPath->udata.tBazierData.tPtNearEnd.dY,
                          itEndStation->tPt.dX,itEndStation->tPt.dY);
        painter->setPen(QPen(QColor(0, 0, 255)));
        painter->fillPath(m_CrtlPointFirst, QBrush(QGradient(QGradient::MagicRay)));
        painter->fillPath(m_CrtlPointSecond, QBrush(QGradient(QGradient::MagicRay)));
    }

    //TestSelfBezier(painter);
}

void CLineItem::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
    emit m_pView->SignalLinePress(m_iPathID);

    if (m_CrtlPointFirst.contains(event->pos()))
    {
        m_bIsPressCtrlPoint1 = true;
        m_bIsDrawCtrlLine = true;
    }
    else if (m_CrtlPointSecond.contains(event->pos()))
    {
        m_bIsPressCtrlPoint2 = true;
        m_bIsDrawCtrlLine = true;
    }
    else if(m_Line.contains(event->pos()))
    {
        m_bIsDrawCtrlLine = true;
    }
    else
    {
        m_bIsDrawCtrlLine = false;
    }
    update();
    QGraphicsItem::mousePressEvent(event);
}

void CLineItem::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{
    QPainterPath temp1, temp2, temp3,temp4;
    auto itPath = CMapManager::GetInstance()->GetPath(m_iPathID);
    auto itStartStation = CMapManager::GetInstance()->GetStation(itPath->iStartStation);
    auto itEndStation = CMapManager::GetInstance()->GetStation(itPath->iEndStation);
    if(m_bIsPressCtrlPoint1 || m_bIsPressCtrlPoint2)
    {
        QPointF qCrtPointFirst, qCrtPointSecond, qStartStation, qEndStation;
        prepareGeometryChange();
        qStartStation.setX(itStartStation->tPt.dX);
        qStartStation.setY(itStartStation->tPt.dY);
        qEndStation.setX(itEndStation->tPt.dX);
        qEndStation.setY(itEndStation->tPt.dY);
        int iStationID = 0;
        if (m_bIsPressCtrlPoint1)
        {
            itPath->udata.tBazierData.tPtNearStart.dX = event->pos().x();
            itPath->udata.tBazierData.tPtNearStart.dY = event->pos().y();
            iStationID = itStartStation->iStationID;

        }
        if (m_bIsPressCtrlPoint2)
        {
            itPath->udata.tBazierData.tPtNearEnd.dX = event->pos().x();
            itPath->udata.tBazierData.tPtNearEnd.dY = event->pos().y();
            iStationID = itEndStation->iStationID;
        }
        std::list<TPath *> listTpath = CMapManager::GetInstance()->GetStationLinkedPath(iStationID);
        if(listTpath.size() >1)
        {
            std::list<TPath *>::iterator plist;
            for (plist = listTpath.begin(); plist != listTpath.end(); plist++)
            {
                TPath * pPathTemp = *plist;
                if(itPath->iPathID == pPathTemp->iPathID || pPathTemp->eType == EPathType::PathType_Circle)
                {
                    continue;
                }
                else
                {
                    TPointd tPos, tIntersectionPt; //鼠标当前点坐标、点到线相交坐标
                    tPos.dX = event->pos().x();
                    tPos.dY = event->pos().y();
                    double dDistance = 0.0;
                    auto itTmpStartStation = CMapManager::GetInstance()->GetStation(pPathTemp->iStartStation);
                    auto itTmpEndStation = CMapManager::GetInstance()->GetStation(pPathTemp->iEndStation);
                    if (iStationID == itTmpStartStation->iStationID) //该站是起始站
                    {
                        dDistance = CGeometryAlgorithm::Point2LineDis(tPos, itTmpStartStation->tPt, pPathTemp->tStartToEndPt, tIntersectionPt);
                    }
                    else  //该站是终止站
                    {
                        dDistance = CGeometryAlgorithm::Point2LineDis(tPos, itTmpEndStation->tPt, pPathTemp->tEndToEndPt, tIntersectionPt);
                    }

                    if(dDistance <= POINTTOLINE_DISTANCE_MIN)
                    {
                        if(m_bIsPressCtrlPoint1)
                        {
                            itPath->udata.tBazierData.tPtNearStart.dX = tIntersectionPt.dX;
                            itPath->udata.tBazierData.tPtNearStart.dY = tIntersectionPt.dY;
                        }
                        if(m_bIsPressCtrlPoint2)
                        {
                            itPath->udata.tBazierData.tPtNearEnd.dX = tIntersectionPt.dX;
                            itPath->udata.tBazierData.tPtNearEnd.dY = tIntersectionPt.dY;
                        }
                    }
                }
            }
        }

        qCrtPointFirst.setX(itPath->udata.tBazierData.tPtNearStart.dX);
        qCrtPointFirst.setY(itPath->udata.tBazierData.tPtNearStart.dY);
        qCrtPointSecond.setX(itPath->udata.tBazierData.tPtNearEnd.dX);
        qCrtPointSecond.setY(itPath->udata.tBazierData.tPtNearEnd.dY);
        temp1.addEllipse(qCrtPointFirst, LINE_CTRLPOINT_LEN, LINE_CTRLPOINT_LEN);
        m_CrtlPointFirst = temp1;
        temp2.addEllipse(qCrtPointSecond, LINE_CTRLPOINT_LEN, LINE_CTRLPOINT_LEN);
        m_CrtlPointSecond = temp2;
        temp3.moveTo(qStartStation);
        temp3.cubicTo(qCrtPointFirst, qCrtPointSecond, qEndStation);
        m_Line = temp3;
        temp4.addPath(m_CrtlPointFirst);
        temp4.addPath(m_CrtlPointSecond);
        temp4.addPath(m_Line);
        m_Line.setFillRule(Qt::WindingFill);
        m_allPath = temp4;
        m_allPath.setFillRule(Qt::WindingFill);
    }
    update();
    QGraphicsItem::mouseMoveEvent(event);
}

void CLineItem::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
    if (m_bIsPressCtrlPoint1)
    {
        m_bIsPressCtrlPoint1 = false;
    }
    if (m_bIsPressCtrlPoint2)
    {
        m_bIsPressCtrlPoint2 = false;
    }
    update();
    QGraphicsItem::mouseReleaseEvent(event);
    emit m_pView->SignalLineModify(m_iPathID);
}

void CLineItem::hoverEnterEvent(QGraphicsSceneHoverEvent *event)
{
    if (m_pView->GetMouseMode() != EMouseMode::MouseMode_Pointer)
    {
        return;
    }
    QGraphicsItem::hoverEnterEvent(event);
    if (isSelected())
    {
        //setCursor(QCursor(Qt::CursorShape::CrossCursor));
        m_bIsHoverEnter = true;
    }
    else
    {
        if (m_Line.contains(event->pos()))
        {
            //setCursor(QCursor(Qt::CursorShape::CrossCursor));
            m_bIsHoverEnter = true;
            qDebug()<<"Enter line";
        }
        else
        {
            m_bIsHoverEnter = false;
            //setCursor(QCursor(Qt::CursorShape::ArrowCursor));
            qDebug()<<"Enter point";
        }
    }
    update();
}

void CLineItem::hoverLeaveEvent(QGraphicsSceneHoverEvent *event)
{
    if (m_pView->GetMouseMode() != EMouseMode::MouseMode_Pointer)
    {
        return;
    }
    QGraphicsItem::hoverLeaveEvent(event);
    m_bIsHoverEnter = false;
    //setCursor(QCursor(Qt::CursorShape::ArrowCursor));
    update();
}

void CLineItem::TestSelfBezier(QPainter *painter)
{
    auto itPath = CMapManager::GetInstance()->GetPath(m_iPathID);
    if(itPath->eType != EPathType::PathType_Bezier )
    {
        return;
    }
    auto itStartStation = CMapManager::GetInstance()->GetStation(itPath->iStartStation);
    auto itEndStation = CMapManager::GetInstance()->GetStation(itPath->iEndStation);

    QPointF pt0 = {(double)itStartStation->tPt.dX,(double) itStartStation->tPt.dY};
    QPointF pt1 = {(double)itPath->udata.tBazierData.tPtNearStart.dX,(double) itPath->udata.tBazierData.tPtNearStart.dY};
    QPointF pt2  {(double)itPath->udata.tBazierData.tPtNearEnd.dX,(double)itPath->udata.tBazierData.tPtNearEnd.dY};
    QPointF pt3 = {(double)itEndStation->tPt.dX,(double) itEndStation->tPt.dY};

//    pt0 += QPointF(50, 0);
//    pt1 += QPointF(50, 0);
//    pt2 += QPointF(50, 0);
//    pt3 += QPointF(50, 0);

    painter->setPen(QPen(QColor(0,0,0), 1));
    for (double i = 0; i<1; i+=0.001)
    {
        QPointF pt = pt0*(1-i)*(1-i)*(1-i) + 3*pt1*i*(1-i)*(1-i) + 3*pt2*i*i*(1-i) + pt3*i*i*i;
        painter->drawPoint(pt);
    }
}

void CLineItem::CalcBezierMidPoint(TPointd tPtStart, TPointd tPt1, TPointd tPt2, TPointd tPtEnd,
                                   QPointF &tPtMid, QPointF &tPtMidToStart, QPointF &tPtMidToEnd)
{
    //计算贝塞尔曲线长度
    TPointd Piont_Bezier[4];                 //存放4个控制点
    Piont_Bezier[0] = tPtStart;
    Piont_Bezier[1] = tPt1;
    Piont_Bezier[2] = tPt2;
    Piont_Bezier[3] = tPtEnd;
    double length = 0.0;

    double a_x = (Piont_Bezier[3].dX - Piont_Bezier[0].dX + 3*Piont_Bezier[1].dX - 3*Piont_Bezier[2].dX);
    double a_y = (Piont_Bezier[3].dY - Piont_Bezier[0].dY + 3*Piont_Bezier[1].dY - 3*Piont_Bezier[2].dY);
    double b_x = 3*(Piont_Bezier[0].dX - 2*Piont_Bezier[1].dX + Piont_Bezier[2].dX);
    double b_y = 3*(Piont_Bezier[0].dY - 2*Piont_Bezier[1].dY + Piont_Bezier[2].dY);
    double c_x = 3*(Piont_Bezier[1].dX-Piont_Bezier[0].dX);
    double c_y = 3*(Piont_Bezier[1].dY-Piont_Bezier[0].dY);
    double a = 9*(a_x*a_x + a_y*a_y);
    double b = 12*(a_x*b_x + a_y*b_y);
    double c = (4*(b_x*b_x + b_y*b_y) + 6*(a_x*c_x + a_y*c_y));
    double d = 4*(b_x*c_x + b_y*c_y);
    double e = (c_x*c_x + c_y*c_y);
    double f_0 = sqrt(e);
    double f_1 = sqrt(a+b+c+d+e);
    double f_1_3 = sqrt(a/81 + b/27 + c/9 + d/3 + e);
    double f_2_3 = sqrt(16*a/81 + 8*b/27 + 4*c/9 + 2*d/3 + e);
    length = (f_0 + 3*f_1_3 + 3*f_2_3 +f_1)/8;

    //进行采样
    double dSim = length/CMapManager::GetInstance()->GetCurrentMapInfo().dResolution; //采样数量
    double dInterval = 1 / dSim;

    double dMid0 = dInterval*dSim/2; //中间点0
    double dMid1 = dInterval*(dSim/2+LINE_ARROW_LENGTH); //中间点0后一个点
    double dMid2 = dInterval*(dSim/2-LINE_ARROW_LENGTH); //中间点0前一个点

    QPointF pt0 = {(double)tPtStart.dX,(double) tPtStart.dY};
    QPointF pt1 = {(double)tPt1.dX,(double) tPt1.dY};
    QPointF pt2  {(double)tPt2.dX,(double) tPt2.dY};
    QPointF pt3 = {(double)tPtEnd.dX,(double) tPtEnd.dY};

    tPtMid = pt0*(1-dMid0)*(1-dMid0)*(1-dMid0) + 3*pt1*dMid0*(1-dMid0)*(1-dMid0) + 3*pt2*dMid0*dMid0*(1-dMid0) + pt3*dMid0*dMid0*dMid0;
    tPtMidToEnd = pt0*(1-dMid1)*(1-dMid1)*(1-dMid1) + 3*pt1*dMid1*(1-dMid1)*(1-dMid1) + 3*pt2*dMid1*dMid1*(1-dMid1) + pt3*dMid1*dMid1*dMid1;
    tPtMidToStart = pt0*(1-dMid2)*(1-dMid2)*(1-dMid2) + 3*pt1*dMid2*(1-dMid2)*(1-dMid2) + 3*pt2*dMid2*dMid2*(1-dMid2) + pt3*dMid2*dMid2*dMid2;
}




