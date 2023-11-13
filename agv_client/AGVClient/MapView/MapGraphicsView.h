#ifndef CMAPGRAPHICSVIEW_H
#define CMAPGRAPHICSVIEW_H

#include <QGraphicsView>
#include <QTimer>
#include <MapView/MapGraphicsScene.h>
#include "MapView/CursorFactory.h"
#include "MapView/MapView_def.h"
#include "AGVClient_def.h"
#include "Commands/CmdMoveStations.h"

class CStationItem;
class CMapGraphicsScene;
class CAgvItem;
class CAGVWidget;
class CLineItem;
class CGlobalPathItem;
class CMapGraphicsView : public QGraphicsView
{
    Q_OBJECT
    friend class CMapGraphicsScene;

public:
    CMapGraphicsView(QGraphicsScene *sceneIn, QWidget *parent = nullptr);

    //绘制当前AGV地图
    void LoadAGVMap();

    //设置鼠标模式
    void SetMouseMode(EMouseMode mode);
    EMouseMode GetMouseMode();

    //设置橡皮擦大小
    void SetEraserSize(int iSize);
    //清除地图孤立点
    void ClearOutlier();
    //重置地图（清除Item及命令队列）
    void ResetMap();

    //AGV掉线
    void AGVDisconnect();

    //AGV实时数据
    void OnUpdateAGVRealTimeData();

    //删除选中的Item
    void DeleteSelectedItems();

    //站点被移动了，修改连接路线的boundingrect
    void StationMove(int iStationID);

    /**
     * @brief AddStationOnAGVPos : 在AGV当前位置增加一个站点
     */
    bool AddStationOnAGVPos();

    /**
     * @brief MoveStationToAGV : 移动选中的站点到AGV当前位置，位姿和AGV一致
     * @return false:未选中站点
     */
    bool MoveStationToAGV(TAGVInfo *pAGV);

    /**
     * @brief 清除Items的选中状态
     */
    void ClearItemsSelectedStatus();

    ///
    /// \brief 增删全局路径Item
    ///
    void AddGlobalPathItem(std::vector<TPointd> &vecPath);
    void DeleGolbalPathItem();


protected:
    void mousePressEvent(QMouseEvent *mouseEvent) override;
    void mouseMoveEvent(QMouseEvent *mouseEvent) override;
    void mouseReleaseEvent(QMouseEvent *mouseEvent) override;
    void wheelEvent(QWheelEvent *event) override;
    void paintEvent(QPaintEvent *event) override;
    void contextMenuEvent(QContextMenuEvent *event) override;

    virtual void enterEvent(QEvent *event);
    virtual void leaveEvent(QEvent *event);
private:
    //两个站点Item之间是否已经有连线
    bool IsTwoStationHaveLine(CStationItem *pStart, CStationItem *pEnd);
    //Path类型转换
    EPathType MouseModeToPathType(EMouseMode mode);
    //绘制实时坐标显示
    void PaintMousePos(QPainter *painter, QPaintEvent *event);
    //绘制世界坐标系xy轴
    void PaintWorldAxis(QPainter *painter, QPaintEvent *event);
    //绘制橡皮擦
    void PaintEraser(QPainter *painter, QPaintEvent *event);
    //删除AGVItem
    void DeleteAGVItem();
    //删除除地图外的所有Item
    void DeleteAllItem();
    //在scene上添加一个站点
    CStationItem *AddStationItem(TStation *ptstation);
    //在scene上添加一个路线
    CLineItem *AddLineItem(int iPathID);

signals:
    //点击了地图
    void SignalMapPress();
    //选中了站点
    void SignalStationPress(int iStationID);
    //编辑了站点位置
    void SignalStationModify(int iStationID);
    //选中了线
    void SignalLinePress(int iLineID);
    //编辑了线
    void SignalLineModify(int iLineID);


private slots:
    void SlotrubberBandChanged(QRect viewportRect, QPointF fromScenePoint, QPointF toScenePoint);

private:
    CAGVWidget *m_pAGVWidget;
    //鼠标模式
    EMouseMode m_eMouseMode;
    //橡皮擦大小
    int m_iEraserSize;
    //是否进入离开view标识 true:进入，false：离开
    bool m_bIsEnterFlg;
    QPainterPath m_pathEraser; //橡皮擦路径
    //地图
    CMapItem *m_pmapItem;
    //创建鼠标光标·
    CCursorFactory m_factory;

    //当前缩放比例
    double m_dScale;

    //画线标记
    bool m_bIsDrawLine;
    //画线起始站点坐标(view坐标系)
    QPointF m_ptStartStation;
    //鼠标当前位置(view坐标系)
    QPointF m_ptCurrent;
    //画线起始站点
    CStationItem *m_pstartStation;
    //AGVItem对象
    CAgvItem *m_pAGVItem;
    //测试定时器
    QTimer m_testTimer;
    //鼠标中键按下
    bool m_bIsMouseMidPress;
    //鼠标左键按下标识
    bool m_bIsMouseLeftPress;

    //是否处在ruberhand选中状态中
    bool m_bIsRuberSelecting;
    //收集选中的站点
    std::list<TMoveStationsData> m_listStationSelected;

    //全局路径Item
    CGlobalPathItem *m_pGlobalItem;

    friend class CAGVWidget;
    friend class CCmdAddStation;
    friend class CStationItem;
    friend class CCmdAddLine;

};

#endif // CMAPGRAPHICSVIEW_H
