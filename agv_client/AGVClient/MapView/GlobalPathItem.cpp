#include "GlobalPathItem.h"
#include <QPainter>

CGlobalPathItem::CGlobalPathItem(CMapGraphicsView *pView, QGraphicsItem *parent)
{

}

void CGlobalPathItem::SetPath(const std::vector<TPointd> &vecPath)
{
    m_vecPath = vecPath;
    update();
}

QRectF CGlobalPathItem::boundingRect() const
{
    return QRectF(0,0,10,10);//不重要
}

void CGlobalPathItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    painter->setRenderHints(QPainter::Antialiasing);//抗锯齿
    //绘制实时局部路径信息
    painter->setPen(QPen(QColor(0, 0, 0), 1, Qt::PenStyle::SolidLine));
    for (auto &it : m_vecPath)
    {
        painter->drawPoint(QPoint(it.dX, it.dY));
    }

}
