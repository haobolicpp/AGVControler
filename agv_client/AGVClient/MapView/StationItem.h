#ifndef CSTATIONITEM_H
#define CSTATIONITEM_H
#include <QGraphicsObject>
#include <QGradient>
#include <QPainterPath>
#include "AGVClient_def.h"
#include "MapView_def.h"

class CStationTextItem;
class CMapGraphicsView;
class CStationItem : public QGraphicsObject //该基类支持信号槽
{
    Q_OBJECT

public:
    CStationItem(int iStationID, CMapGraphicsView *pView, QGraphicsItem *parent = nullptr);

    //设置绑定的CStationTextItem
    void SetStationTextItem(CStationTextItem *pItem){m_pTextItem = pItem;};
    CStationTextItem *GetTextItem(){return m_pTextItem;};

    //鼠标移动，从scene传入，代替mouseMoveEvent，后者只能按下鼠标移动时响应.
    //传入的pos为scene下的pos坐标
    void MouseMove(QPointF pos);

    /**
     * @brief GetStation
     */
    int GetStation(){return m_iStationID;}

protected:
    enum { Type = ITEM_STATION };
    int type() const override { return Type; }
    QRectF boundingRect() const override;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget = nullptr) override;
    virtual void mousePressEvent(QGraphicsSceneMouseEvent *event) override;
    virtual void mouseMoveEvent(QGraphicsSceneMouseEvent *event) override;
    virtual void mouseReleaseEvent(QGraphicsSceneMouseEvent *event) override;
    void hoverEnterEvent(QGraphicsSceneHoverEvent *event) override;
    //void hoverMoveEvent(QGraphicsSceneHoverEvent *event) override;
    void hoverLeaveEvent(QGraphicsSceneHoverEvent *event) override;

private:
    //显示的轮廓
    QPainterPath m_borderPath;
    //控制点
    QPainterPath m_controlPointPath;
    //总体轮廓
    QPainterPath m_allPath;
    //显示文本的Item
    CStationTextItem *m_pTextItem;
    //绑定的站点
    int m_iStationID;
    //是否鼠标进入
    bool m_bIsHoverEnter;
    //鼠标是否在控制点按下
    bool m_bIsPressInCrontrolPoint;
    //鼠标按下时候的变换矩阵
    QMatrix m_matrix;
    //鼠标按下旋转Item的坐标
    QPointF m_ptCurrent;
    //view
    CMapGraphicsView *m_pView;
    //移动前站点文本的位置
    QPointF m_tLastTxtPos;
    //移动前站点的位置
    QPointF m_tLastStationPos;
    //旋转站点前的角度
    double m_dLastAngle;
    //是否在站点中按下过
    bool m_bIsPressInItem;

    friend class CMapGraphicsView;
    friend class CCmdModifyStation;
    friend class CCmdMoveStations;

};

#endif // CSTATIONITEM_H
