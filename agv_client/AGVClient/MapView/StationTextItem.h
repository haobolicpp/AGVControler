#ifndef CSTATIONTEXTITEM_H
#define CSTATIONTEXTITEM_H
#include <QGraphicsTextItem>
#include "AGVClient_def.h"
#include "MapView_def.h"

class CStationItem;
class CStationTextItem : public QGraphicsTextItem
{
public:
    CStationTextItem(int iStationID, QGraphicsItem *parent = nullptr);

    //根据绑定的StationItem, 更新当前Item的位置
    //pos为绑定的StationItem在Scene中的坐标
    void MoveItemByStationItem(QPointF pos);

protected:
    enum { Type = ITEM_STATION_TXT };
    int type() const override { return Type; }

private:
    int m_iStationID;
};

#endif // CSTATIONTEXTITEM_H
