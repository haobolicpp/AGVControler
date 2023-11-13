#include <QPainter>
#include <QStyleOptionGraphicsItem>
#include <QPolygonF>
#include <QDebug>
#include <QGraphicsSceneHoverEvent>
#include "AgvItem.h"
#include "MapManager.h"
#include "QGraphicsScene"

CAgvItem::CAgvItem(QGraphicsItem *parent):
    QGraphicsItem(parent)
{
    m_dLastAngle = -90;
    m_bIsHoverEnter = false;

    //轮廓形状，初始按车头朝向-90°绘制
    m_borderPath.moveTo(-AGV_WIDTH/2, -AGV_HIGHT/2);
    m_borderPath.lineTo(-AGV_WIDTH/2, AGV_HIGHT/2);
    m_borderPath.lineTo(AGV_WIDTH/2, AGV_HIGHT/2);
    m_borderPath.lineTo(AGV_WIDTH/2, -AGV_HIGHT/2);
    m_borderPath.lineTo(-AGV_WIDTH/2, -AGV_HIGHT/2);
    //绘制竖线
    QPainterPath pathTemp1, pathTemp2, pathTemp3;
    pathTemp1.moveTo(0, -AGV_HIGHT/2);
    pathTemp1.lineTo(0, 0);
    m_borderPath.addPath(pathTemp1);
    //绘制轮子
    pathTemp2.moveTo(-AGV_WIDTH/2, -AGV_HIGHT/4);
    pathTemp2.lineTo(-AGV_WIDTH/4, -AGV_HIGHT/4);
    pathTemp2.lineTo(-AGV_WIDTH/4, AGV_HIGHT/4);
    pathTemp2.lineTo(-AGV_WIDTH/2, AGV_HIGHT/4);
    m_wheelPath.addPath(pathTemp2);
    pathTemp3.moveTo(AGV_WIDTH/2, -AGV_HIGHT/4);
    pathTemp3.lineTo(AGV_WIDTH/4, -AGV_HIGHT/4);
    pathTemp3.lineTo(AGV_WIDTH/4, AGV_HIGHT/4);
    pathTemp3.lineTo(AGV_WIDTH/2, AGV_HIGHT/4);
    m_wheelPath.addPath(pathTemp3);
    m_borderPath.addPath(m_wheelPath);

    //标记
    setFlags(/*QGraphicsItem::ItemIsMovable | */QGraphicsItem::ItemIsSelectable
             |QGraphicsItem::ItemSendsGeometryChanges);
    setAcceptHoverEvents(true);//可激发hoverXXXEvent回调
}

QRectF CAgvItem::boundingRect() const
{
    return m_borderPath.boundingRect();
}

void CAgvItem::MouseMove(QPointF pos)
{
}

void CAgvItem::UpdateAGVItem(TAGVInfo *pAGVInfo)
{
    //平移加旋转
    QTransform trans = transform();
    QPointF tPtNow;

    CMapManager::GetInstance()->WorldToMap(pAGVInfo->tPt.dX, pAGVInfo->tPt.dY, tPtNow.rx(), tPtNow.ry());
    //qDebug() << "MapPt:" << tPtNow.iX << "," << tPtNow.iY;
    //注意，必须先平移再旋转（待确定）
    setPos(tPtNow.x(), tPtNow.y()); //这个直接平移就可以，下面有点问题
    //trans.translate(tPtNow.iX-tPtLast.iX, tPtNow.iY-tPtLast.iY);
    double diff = pAGVInfo->dAngle - m_dLastAngle;
    trans.rotate(diff); //顺时针旋转坐标系，因为y轴向下，所以顺时针旋转
    setTransform(trans);

    m_dLastAngle = pAGVInfo->dAngle;

    //局部路径坐标转换
    m_vecPath.clear();
    for (auto &pt : pAGVInfo->vecDWALocalPath)
    {
        double dx, dy;
        CMapManager::GetInstance()->WorldToMap(pt.dX, pt.dY, dx, dy);
        m_vecPath.push_back(QPointF{dx, dy});
    }

}

void CAgvItem::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
    QGraphicsItem::mousePressEvent(event);
}

void CAgvItem::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{
    QGraphicsItem::mouseMoveEvent(event);
}

void CAgvItem::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
    QGraphicsItem::mouseReleaseEvent(event);
}

void CAgvItem::hoverEnterEvent(QGraphicsSceneHoverEvent *event)
{
    QGraphicsItem::hoverEnterEvent(event);
    m_bIsHoverEnter = true;
    update();
}

void CAgvItem::hoverLeaveEvent(QGraphicsSceneHoverEvent *event)
{
    QGraphicsItem::hoverLeaveEvent(event);
    m_bIsHoverEnter = false;
    update();
}

void CAgvItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    QColor colorBackGround;
    int ipenWidth = 1;
    Qt::PenStyle penStyle;
    painter->setRenderHints(QPainter::Antialiasing);//抗锯齿

    //背景色
    double dOpacity = 1.0;//不透明度
    if ((option->state & QStyle::State_Selected) || m_bIsHoverEnter)
    {
        colorBackGround = AGV_COLOR_SELECTED;
    }
    else
    {
        colorBackGround = AGV_COLOR;
        dOpacity = 0.7;
    }
    ipenWidth = 1;
    penStyle = Qt::PenStyle::SolidLine;
    painter->setOpacity(dOpacity);
    painter->setBrush(colorBackGround);

    //绘制轮廓
    painter->setPen(QPen(QColor(0, 0, 0), ipenWidth, penStyle));
    painter->drawPath(m_borderPath);

    //绘制轮子
    painter->setBrush(AGV_COLOR_WHEEL);
    painter->drawPath(m_wheelPath);

    //绘制实时局部路径信息
    painter->setPen(QPen(QColor(200, 200, 0), 1, Qt::PenStyle::SolidLine));
    for (auto &it : m_vecPath)
    {
        painter->drawPoint(it);
    }

}
