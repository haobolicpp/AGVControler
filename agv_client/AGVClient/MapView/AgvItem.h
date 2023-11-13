#ifndef CAGVITEM_H
#define CAGVITEM_H

#include <QGraphicsItem>
#include <QPainterPath>
#include "MapView_def.h"
#include "AGVClient_def.h"

class CAgvItem : public QGraphicsItem
{
public:
    CAgvItem(QGraphicsItem *parent = nullptr);
    enum { Type = ITEM_AGV };
    int type() const override { return Type; }

    //鼠标移动，从scene传入，代替mouseMoveEvent，后者只能按下鼠标移动时响应.
    //传入的pos为scene下的pos坐标
    void MouseMove(QPointF pos);

    //更新AGV数据
    void UpdateAGVItem(TAGVInfo *pAGVInfo);
protected:
    QRectF boundingRect() const override;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget = nullptr) override;
    virtual void mousePressEvent(QGraphicsSceneMouseEvent *event) override;
    virtual void mouseMoveEvent(QGraphicsSceneMouseEvent *event) override;
    virtual void mouseReleaseEvent(QGraphicsSceneMouseEvent *event) override;
    void hoverEnterEvent(QGraphicsSceneHoverEvent *event) override;
    void hoverLeaveEvent(QGraphicsSceneHoverEvent *event) override;
private:
    //显示的轮廓
    QPainterPath m_borderPath;
    //两个轮子
    QPainterPath m_wheelPath;
    //是否鼠标进入
    bool m_bIsHoverEnter;
    //实时路径信息
    std::vector<QPointF> m_vecPath;
    //上次收到坐标时的角度
    double m_dLastAngle;
};

#endif // CAGVITEM_H
