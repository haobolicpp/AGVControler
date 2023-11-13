#include <QFont>
#include <QGraphicsSceneHoverEvent>
#include "StationTextItem.h"

CStationTextItem::CStationTextItem(int iStationID, QGraphicsItem *parent):
    QGraphicsTextItem(parent)
{
    m_iStationID = iStationID;

    QFont font("楷体", 6, QFont::Normal);
    setFont(font);
    setPlainText(QString("%1").arg(m_iStationID));

    setAcceptHoverEvents(false); //关键！！默认是开启的，会导致进入Station时先响应本Item（因为本Item在上层）

}

void CStationTextItem::MoveItemByStationItem(QPointF pos)
{
    setPos(pos);
    moveBy(-boundingRect().width()/2, -boundingRect().height()/2);
}

