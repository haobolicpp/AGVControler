#ifndef CMAPITEM_H
#define CMAPITEM_H
#include <QGraphicsItem>
#include <QPolygonF>
#include <QPainterPath>
#include "MapView_def.h"

class CMapGraphicsView;
class CMapItem : public QGraphicsItem
{

public:
    CMapItem(CMapGraphicsView *pView, QGraphicsItem *parent = nullptr);

    /**
     * @brief UpdatePainterMap : 修改地图绘图数据(提前将地图画到pixmap中，这样在item的paint()频繁更新时，不会占用太多时间)
     * @param bScanmap : 是否是扫描地图
     */
    void UpdatePainterMap(bool bScanmap);

protected:
    enum { Type = ITEM_MAP };
    int type() const override { return Type; }
    QRectF boundingRect() const override;//QGraphicsView 判断是否重绘的区域
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget = nullptr) override;
    virtual void mousePressEvent(QGraphicsSceneMouseEvent *event) override;

    //返回一个闭合的路径,The shape is used for many things, including collision detection, hit tests
    //QPainterPath shape() const override;
    //QGraphicsView调用判断当前item是否在某个点下面，会调用比较频繁，区域默认调用shape()来获得.但可重载自己实现,
    //bool contains(const QPointF &point) const override;
    //bool isObscuredBy(const QGraphicsItem *item) const override; //是否被其他item覆盖
    //QPainterPath opaqueArea() const override;//返回不透明闭合区域
private:

private:
    //地图区域
    QPainterPath m_painterPath;
    //view
    CMapGraphicsView *m_pView;
    //QPixmap,加一个缓冲层提速
    QPixmap *m_pPixmap;
};

#endif // CMAPITEM_H
