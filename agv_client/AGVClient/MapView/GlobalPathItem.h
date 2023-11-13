/*
 * 全局路径Item
*/
#ifndef CGLOBALPATH_H
#define CGLOBALPATH_H
#include <QGraphicsItem>
#include <QPainterPath>
#include "MapView_def.h"
#include "AGVClient_def.h"

class CMapGraphicsView;

class CGlobalPathItem : public QGraphicsItem
{
public:
    CGlobalPathItem(CMapGraphicsView *pView, QGraphicsItem *parent = nullptr);

    enum { Type = ITEM_GLOBALPATH };
    int type() const override { return Type; }

    void SetPath(const std::vector<TPointd> &vecPath);

protected:
    QRectF boundingRect() const override;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget = nullptr) override;

    std::vector<TPointd> m_vecPath;
};

#endif // CGLOBALPATH_H
