#ifndef CLINEITEM_H
#define CLINEITEM_H
#include <QGraphicsItem>
#include <QPainterPath>
#include "MapView_def.h"
#include "AGVClient_def.h"

class CMapGraphicsView;
class CLineItem : public QGraphicsItem
{
public:
    CLineItem(int iPahtID, CMapGraphicsView *pView, QGraphicsItem *parent = nullptr);
    enum { Type = ITEM_LINE };
    int type() const override { return Type; }

    //鼠标移动，从scene传入，代替mouseMoveEvent，后者只能按下鼠标移动时响应.
    //传入的pos为scene下的pos坐标
    void MouseMove(QPointF pos);

    /**
     * @brief StationMove : 站点移动，更新boundingrect标记
     */
    void StationMove();

    /**
     * @brief GetPath
     * @return
     */
    //TPath *GetPath(){return m_pPath;};
    int GetPath(){return m_iPathID;}
protected:
    QRectF boundingRect() const override;
    QPainterPath shape() const override;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget = nullptr) override;
    virtual void mousePressEvent(QGraphicsSceneMouseEvent *event) override;
    virtual void mouseMoveEvent(QGraphicsSceneMouseEvent *event) override;
    virtual void mouseReleaseEvent(QGraphicsSceneMouseEvent *event) override;
    void hoverEnterEvent(QGraphicsSceneHoverEvent *event) override;
    void hoverLeaveEvent(QGraphicsSceneHoverEvent *event) override;


    //测试自绘的贝塞尔曲线
    void TestSelfBezier(QPainter *painter);
    //计算贝塞尔曲线中间点的位置、中间点到起点方向附近点、中间点到终点方向附近点
    void CalcBezierMidPoint(TPointd tPtStart, TPointd tPt1, TPointd tPt2, TPointd tPtEnd,
                            QPointF &tPtMid, QPointF &tPtMidToStart, QPointF &tPtMidToEnd);
private:
    //路线数据
    //TPath *m_pPath;
    int m_iPathID; //路线Id
    //控制点
    QPainterPath m_CrtlPointFirst;
    QPainterPath m_CrtlPointSecond;
    //贝塞尔曲线
    QPainterPath m_Line;
    //控制点+线
    QPainterPath m_allPath;

    //临时直线
    QPainterPath m_LineTemp;
    //临时贝塞尔曲线
    QPainterPath m_BezierTemp;
    //是否鼠标进入
    bool m_bIsHoverEnter;
    //鼠标是否在控制点按下
    bool m_bIsPressCtrlPoint1;
    bool m_bIsPressCtrlPoint2;
    //是否画控制点及其连接线
    bool m_bIsDrawCtrlLine;
    //view
    CMapGraphicsView *m_pView;

    friend class CMapGraphicsView;
};

#endif // CLINEITEM_H
