/*【总结】
 * 1.View坐标系(0,0)位于窗口左上角,x向右，y向下；
 * 2.Scene可以通过setSceneRect设置区域，(0,0)位于此区域中心位置，rect区域左上角和View原点重合！也可以不设置区域，此时区域会根据item自动调整，
 * 如本案例中添加地图后，地图Item和Scene的原点是重合的。
 * 3.Item类似Scene，boundingrect描述区域，(0,0)位于区域中心。
 * 4.当放大到出现滚动条时，此时会qt框架会将Scene的中心原点移动到View的左上角，此时图元位置会发生跳变，
 * 注意垂直滚动条出现的时候，Scene的中心原点y会被平移到View最上面，水平滚动条出现的时候，Scene的中心唁电x会被
 * 平移到View的最左面，只有到两个滚动条同时出现时，才出现Scene和View重合的情况。
 * 5.在View中调用updae()时，paintevent不会实时触发，改为viewport->update()就可以了
 * 6.在view的paintevent重载中，必须将QGraphicsView::paintEvent(event)放到头部，否在在view上的绘图会被scene覆盖，同时qpainter对象须传递viewport指针。
 * 7.和update类似，必须调用viewport()->setcursor()才能正确设置鼠标指针，否在会出现莫名其妙的问题。
*/
#include <set>
#include <QWheelEvent>
#include <QScrollBar>
#include <QApplication>
#include <QGraphicsRectItem>
#include <QDebug>
#include <QGraphicsSceneMouseEvent>
#include <QGraphicsScene>
#include <QStaticText>
#include <QMenu>
#include <math.h>
#include <QtMath>
#include "MapItem.h"
#include "StationItem.h"
#include "MapView_def.h"
#include "StationTextItem.h"
#include "MapGraphicsView.h"
#include "MapManager.h"
#include "LineItem.h"
#include "GlogWrapper.h"
#include "AgvItem.h"
#include "AGVManager.h"
#include "CoreControl.h"
#include "Commands/CommandManager.h"
#include "Commands/CmdAddStation.h"
#include "Commands/CmdDelete.h"
#include "Commands/CmdModifyStation.h"
#include "Commands/CmdMoveStations.h"
#include "Commands/CmdAddLine.h"
#include "AGVWindow/AGVWidget.h"
#include "MapView/GlobalPathItem.h"

CMapGraphicsView::CMapGraphicsView(QGraphicsScene *sceneIn, QWidget *parent)    :
    QGraphicsView(sceneIn, parent)
{
    if (parent == nullptr)
    {
        return;
    }
    m_pGlobalItem = nullptr;
    m_pAGVWidget = (CAGVWidget*)parent;
    m_bIsMouseMidPress = false;
    m_bIsDrawLine = false;
    m_pAGVItem = nullptr;
    m_bIsRuberSelecting = false;
    m_bIsEnterFlg = false;
    m_bIsMouseLeftPress = false;
    m_iEraserSize = 1;
    //setScene(sceneIn);
    centerOn(0,0);
    //缩放设置
    m_dScale = 1.0;
    setTransformationAnchor(QGraphicsView::AnchorUnderMouse); //围绕鼠标缩放，帮助中提到，当有滚动条出现时起作用，没有滚动条时自适应。
    setResizeAnchor(QGraphicsView::AnchorUnderMouse);
    setDragMode(QGraphicsView::DragMode::RubberBandDrag);
    setOptimizationFlags(QGraphicsView::DontSavePainterState);
    setViewportUpdateMode(QGraphicsView::FullViewportUpdate);
    setMouseTracking(true);//不设置此项，鼠标只有在按下时才会相应mousemove
    //设置背景色
    setBackgroundBrush(QBrush(QColor(61,61,61)));

    setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
    setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOn);

    //右键菜单
    setContextMenuPolicy(Qt::DefaultContextMenu);

    //绑定鼠标选中信号
    connect(this, &CMapGraphicsView::rubberBandChanged, this, &CMapGraphicsView::SlotrubberBandChanged);

    //测试
    m_pmapItem = new CMapItem(this);
    m_pmapItem->setPos(0,0);
    scene()->addItem(m_pmapItem);
    connect(&m_testTimer, &QTimer::timeout, [&](){
        if (m_pAGVItem == nullptr) return ;
        QTransform trans = m_pAGVItem->transform();
        static TPoint tPtNow{0,0}, tPtLast{0,0};
        static double dNowAngle(-90), dLastAngle(-90);
        //tPtNow.iX+=10;
        //tPtNow.iY+=10;
        dNowAngle+=20;
        //注意，必须先平移再旋转
        trans.translate(tPtNow.iX-tPtLast.iX, tPtNow.iY-tPtLast.iY);
        trans.rotate(dNowAngle-dLastAngle); //顺时针旋转坐标系，因为y轴向下，所以顺时针旋转
        m_pAGVItem->setTransform(trans);
        tPtLast = tPtNow;
        dLastAngle = dNowAngle;
    });
    //m_testTimer.start(1000);

}

void CMapGraphicsView::LoadAGVMap()
{
    m_listStationSelected.clear();

    SetMouseMode(EMouseMode::MouseMode_Pointer); //不设置缩放有问题 TODO 缩放还有别的问题
    //测试
//    CAgvItem *pAGVItem = new CAgvItem();
//    m_pAGVItem = pAGVItem;
//    scene()->addItem(pAGVItem);
    TMapInfo &tMapInfo = CMapManager::GetInstance()->GetCurrentMapInfo();

    //更新此时的地图数据
    m_pmapItem->UpdatePainterMap(false);
    //删除所有站点及路线、AGV
    DeleteAllItem();
    //增加Item
    //新增站点
    for (auto &itStation : tMapInfo.mapStation)
    {
        AddStationItem(&itStation.second);
    }
    //新增路线
    for (auto &itPath : tMapInfo.mapPath)
    {
        AddLineItem(itPath.second.iPathID);
    }
    //新增AGV
    std::string strAGVID = CCoreControl::GetInstance()->GetCurrentMapAGVID();
    TAGVInfo *pAGVInfo = CAGVManager::GetInstance()->GetAGV(strAGVID);
    if (pAGVInfo->eStatus == EConnectStatus::Connected_Sucess)
    {
        CAgvItem *pAGVItem = new CAgvItem();
        m_pAGVItem = pAGVItem;
        m_pAGVItem->setZValue(1);
        scene()->addItem(pAGVItem);
        pAGVItem->setPos(0,0);
    }

    this->viewport()->update();
}

void CMapGraphicsView::SetMouseMode(EMouseMode mode)
{
    m_eMouseMode = mode;

    if (m_eMouseMode == EMouseMode::MouseMode_Pointer){
       setDragMode(QGraphicsView::RubberBandDrag);
    }else{
        setDragMode(QGraphicsView::NoDrag);
    }

    //光标设置
    switch (m_eMouseMode) {
    case EMouseMode::MouseMode_Pointer:
        viewport()->setCursor(Qt::CursorShape::ArrowCursor);
        break;
    case EMouseMode::MouseMode_Station:
        viewport()->setCursor(m_factory.CreateCursor(ECursor::Cursor_Station));
        break;
    case EMouseMode::MouseMode_Line:
    case EMouseMode::MouseMode_Arc:
    case EMouseMode::MouseMode_Bezier:
        viewport()->setCursor(m_factory.CreateCursor(ECursor::Cursor_Line));
        break;
    case EMouseMode::MouseMode_Eraser:
        viewport()->setCursor(m_factory.CreateCursor(ECursor::Cursor_Eraser));
        break;
    case EMouseMode::MouseMode_ToDes:
        viewport()->setCursor(m_factory.CreateCursor(ECursor::Cursor_Des));
        break;
    default:
        break;
    }
}

EMouseMode CMapGraphicsView::GetMouseMode()
{
    return m_eMouseMode;
}

void CMapGraphicsView::SetEraserSize(int iSize)
{
    m_iEraserSize = iSize;
}

void CMapGraphicsView::ClearOutlier()
{
    TMapInfo &mapinfo = CMapManager::GetInstance()->GetCurrentMapInfo();
    std::vector<TPoint>::iterator iterPoint;
    std::vector<TPoint>::iterator iterPoint2;
    for (iterPoint=mapinfo.vecOccupy.begin();iterPoint!=mapinfo.vecOccupy.end();)
    {
        for(iterPoint2=mapinfo.vecOccupy.begin();iterPoint2!=mapinfo.vecOccupy.end();iterPoint2++)
        {
            int deltaX = iterPoint2->iX - iterPoint->iX;
            int deltaY = iterPoint2->iY - iterPoint->iY;
            if((0 == deltaX) && (0 == deltaY))
            {
                continue;
            }
            double dis = std::sqrt(deltaX*deltaX + deltaY*deltaY);
            if(dis < CLEAROUTLIER)
            {
                ++iterPoint;
                break;
            }
        }
        if(iterPoint2 ==mapinfo.vecOccupy.end())
        {
            iterPoint = mapinfo.vecOccupy.erase(iterPoint);
        }
    }
    m_pmapItem->UpdatePainterMap(false);
    viewport()->update();
}

void CMapGraphicsView::ResetMap()
{
    CCommandManager::GetInstance()->Clear();//清除命令
    //清空scene中的路线、站点Item
    QList<QGraphicsItem *> listItems = items();
    for (auto &it : listItems)
    {
        if (it->type()==ITEM_LINE || it->type()==ITEM_STATION || it->type()==ITEM_STATION_TXT)
        {
            scene()->removeItem(it);
            delete it;
        }
    }
}

void CMapGraphicsView::AGVDisconnect()
{
    DeleteAGVItem();
    DeleGolbalPathItem();
    this->viewport()->update();
}

void CMapGraphicsView::OnUpdateAGVRealTimeData()
{
    if (m_pAGVItem != nullptr)
    {
        std::string strAGVID = CCoreControl::GetInstance()->GetCurrentMapAGVID();
        TAGVInfo *pInfo = CAGVManager::GetInstance()->GetAGV(strAGVID);
        m_pAGVItem->UpdateAGVItem(pInfo);
    }
}

void CMapGraphicsView::DeleteSelectedItems()
{
    std::set<CLineItem *> setLine;
    std::set<CStationItem *> setStation;

    //删除站点，收集连接的路线
    std::list<TPath *> listLinkPath; //站点连接的路线
    QList<QGraphicsItem *> listItems = items();
    for (auto &it : listItems)
    {
        if (it->isSelected() && it->type() == ITEM_STATION)
        {
            std::list<TPath *> listPath = CMapManager::GetInstance()->GetStationLinkedPath(((CStationItem*)it)->GetStation());
            listLinkPath.merge(listPath);
            setStation.insert((CStationItem*)it);
        }
    }

    //清空收集的路线
    QList<QGraphicsItem *> listItems2 = items();
    for (auto &itPathItem : listItems2)
    {
        if (itPathItem->type() == ITEM_LINE)
        {
            for (auto &itPath : listLinkPath)
            {
                if (((CLineItem*)itPathItem)->m_iPathID == itPath->iPathID)
                {
                    setLine.insert((CLineItem*)itPathItem);
                    break;
                }
            }
        }
    }

    //清空选中的路线
    QList<QGraphicsItem *> listItems3 = items();
    for (auto &it : listItems3)
    {
        if (it->isSelected())
        {
            if (it->type() == ITEM_LINE)
            {
                setLine.insert((CLineItem*)it);
            }
        }
    }

    //执行删除指令
    CCmdDelete *pCmd = new CCmdDelete(this, setStation, setLine);
    CCommandManager::GetInstance()->AddCmd(pCmd);
    pCmd->exec();
 }

void CMapGraphicsView::mousePressEvent(QMouseEvent *mouseEvent)
{
    if (mouseEvent->button() == Qt::LeftButton)
    {
        if (m_eMouseMode == EMouseMode::MouseMode_Station)
        {
            //在鼠标当前位置添加一个站点
            QPointF ptf = mapToScene(mouseEvent->pos());
            CCmdBase *pCmd = new CCmdAddStation(this, ptf, -90, EStationType::StationType_General);
            CCommandManager::GetInstance()->AddCmd(pCmd);
            pCmd->exec();

        }
        else if (m_eMouseMode==EMouseMode::MouseMode_Line || m_eMouseMode==EMouseMode::MouseMode_Arc
                  || m_eMouseMode==EMouseMode::MouseMode_Bezier)
        {
            QList<QGraphicsItem *> list = items(mouseEvent->pos());
            for (auto it : list){
                if (it->type() ==ITEM_STATION){//点击站点需画线
                    m_bIsDrawLine = true;
                    m_ptStartStation = mapFromScene(it->scenePos());
                    m_pstartStation = (CStationItem*)it;
                    break;
                }
            }
        }
        else if (m_eMouseMode == EMouseMode::MouseMode_Pointer)
        {
            QList<QGraphicsItem *> listItems = items(mouseEvent->pos());
            if ((listItems.size() == 1 && listItems.at(0)->type()==ITEM_MAP) || listItems.empty())
            {
                m_bIsRuberSelecting = true; //什么Item都不点，认为选中操作开始了
            }
        }

        m_bIsMouseLeftPress = true;
    }
    else if (mouseEvent->button() == Qt::RightButton)
    {


    }
    else if (mouseEvent->button() == Qt::MiddleButton)
    {
        setDragMode(QGraphicsView::ScrollHandDrag);
        setInteractive(false);
        m_bIsMouseMidPress = true;
        //模拟鼠标按下,实现左键按下抓住的功能
        QMouseEvent event0(QEvent::MouseButtonPress, mouseEvent->pos(), Qt::LeftButton, Qt::LeftButton, Qt::NoModifier);
        QGraphicsView::mousePressEvent(&event0);
    }

    //只有指针模式、去目标站模式才会响应其他Item的按下操作
    if(EMouseMode::MouseMode_Pointer == m_eMouseMode || EMouseMode::MouseMode_ToDes == m_eMouseMode){
        QGraphicsView::mousePressEvent(mouseEvent);
    }
}

void CMapGraphicsView::mouseMoveEvent(QMouseEvent *mouseEvent)
{
    m_ptCurrent = mouseEvent->pos();

    if (m_bIsMouseMidPress)
    {
        QGraphicsView::mouseMoveEvent(mouseEvent);
        return;
    }

    if(m_eMouseMode == EMouseMode::MouseMode_Eraser && m_bIsMouseLeftPress)
    {
        //如果鼠标左键按下则进行橡皮擦清除
        //获取地图数据
        TMapInfo &mapinfo = CMapManager::GetInstance()->GetCurrentMapInfo();
        std::vector<TPoint>::iterator iterPoint;

        for (iterPoint=mapinfo.vecOccupy.begin();iterPoint!=mapinfo.vecOccupy.end();)
        {
            QPointF qEraserPoint{(qreal)iterPoint->iX, (qreal)iterPoint->iY};

            TPoint tpt;
            CMapManager::GetInstance()->ColRowPixmapToScene(iterPoint->iY, iterPoint->iX, tpt.iY, tpt.iX); //pix坐标转scene
            QPointF pt{(qreal)tpt.iX, (qreal)tpt.iY};
            pt = mapFromScene(pt);// 场景转view
            if(m_pathEraser.contains(pt))
            {
                iterPoint = mapinfo.vecOccupy.erase(iterPoint);
            }
            else
            {
                ++iterPoint;
            }
        }
        m_pmapItem->UpdatePainterMap(false);
        viewport()->update();
        //QGraphicsView::mouseMoveEvent(mouseEvent);
        return;
    }

    if (m_eMouseMode!=EMouseMode::MouseMode_Pointer)
    {
        //非指针模式 例如站点会对其它item产生影响
        viewport()->update();
        //QGraphicsView::mouseMoveEvent(mouseEvent);
        return;
    }

    if (m_eMouseMode == EMouseMode::MouseMode_Pointer){

        if (CCoreControl::GetInstance()->GetAGVRunStatus() == EAGVRunStatus::Run_GoToDes)
        {
            //导航时直接返回
            return;
        }
        if (!CCoreControl::GetInstance()->GetEditMap())
        {
            //非编辑地图页面，返回
            return;
        }

        //判断移动到哪个Item上
        QList<QGraphicsItem *> list = items(mouseEvent->pos());
        for (auto it : list){
            if (it->type() ==ITEM_STATION){
                ((CStationItem*)it)->MouseMove(mapToScene(mouseEvent->pos())); //调用接口
            }else if (it->type() == ITEM_LINE)
            {
                ((CLineItem*)it)->MouseMove(mapToScene(mouseEvent->pos())); //调用接口
            }
            break;
        }
    }

    //更新鼠标所在点的坐标
    this->viewport()->update();

    QGraphicsView::mouseMoveEvent(mouseEvent);
}

void CMapGraphicsView::mouseReleaseEvent(QMouseEvent *mouseEvent)
{
    if (mouseEvent->button() == Qt::MiddleButton)
    {
        //【巨坑】需要首先设置NoDrag来清空qt中view父类中的拖拽标记
        setDragMode(QGraphicsView::NoDrag);
        //重新设置当前鼠标模式
        SetMouseMode(m_eMouseMode);
        setInteractive(true);
        m_bIsMouseMidPress = false;
        QGraphicsView::mouseReleaseEvent(mouseEvent);
        return ;
    }
    if (m_bIsDrawLine){
        m_bIsDrawLine = false;
        this->viewport()->update();
        //判断是否在站点item上抬起，添加一个线段
        QList<QGraphicsItem *> list = items(mouseEvent->pos());
        for (auto it : list){
            if (it->type() == ITEM_STATION && m_pstartStation->GetStation()!=((CStationItem*)it)->GetStation()){
                //判断起始站点和终止站点之间是否有连线
                if (!IsTwoStationHaveLine(m_pstartStation, (CStationItem*)it)){
                    //添加线
                    CCmdBase *pCmd = new CCmdAddLine(this, MouseModeToPathType(m_eMouseMode), m_pstartStation->GetStation(), ((CStationItem*)it)->GetStation());
                    CCommandManager::GetInstance()->AddCmd(pCmd);
                    pCmd->exec();
                    break;
                }else{

                }
            }
        }
    }
    else
    {
        if (mouseEvent->button() == Qt::LeftButton)
        {
            if (m_bIsRuberSelecting)
            {
                m_listStationSelected.clear();
                //寻找选中的站点
                QList<QGraphicsItem *> listItems = items();
                for (auto &it : listItems)
                {
                    if (it->type()==ITEM_STATION && it->isSelected())
                    {
                        TMoveStationsData tData;
                        tData.pItem = (CStationItem*)it;
                        tData.tLastStationPt = it->pos();
                        tData.tLastStationTxtPt = ((CStationItem*)it)->GetTextItem()->pos();
                        m_listStationSelected.push_back(tData);
                    }
                }
                m_bIsRuberSelecting = false;
            }
            else
            {

            }

            m_bIsMouseLeftPress = false;
        }
    }
    QGraphicsView::mouseReleaseEvent(mouseEvent);
}

void CMapGraphicsView::wheelEvent(QWheelEvent *event)
{
    QMatrix matrix;
    static double d = 1.0;
    if (event->modifiers() & Qt::ControlModifier) {
        if (event->delta() > 0){
            d += 0.1;
        }else{
            d -= 0.1;
            if (d <= 0.1) d = 0.1;
        }
        matrix.scale(d, d);
        setMatrix(matrix);
        event->accept();//关键，不屏蔽则会和滚动条滚动冲突
    } else {
        QGraphicsView::wheelEvent(event);
    }
}

bool CMapGraphicsView::IsTwoStationHaveLine(CStationItem *pStart, CStationItem *pEnd)
{
    const TMapInfo &mapinfo = CMapManager::GetInstance()->GetCurrentMapInfo();
    for (auto &it : mapinfo.mapPath){
        if ((it.second.iStartStation==pStart->GetStation() &&
             it.second.iEndStation==pEnd->GetStation()) ||
             (it.second.iStartStation==pEnd->GetStation() &&
              it.second.iEndStation==pStart->GetStation())){
            return true;
        }
    }
    return false;
}

EPathType CMapGraphicsView::MouseModeToPathType(EMouseMode mode)
{
    EPathType type = EPathType::PathType_Line;
    switch (mode) {
    case EMouseMode::MouseMode_Arc:
        type = EPathType::PathType_Circle;
        break;
    case EMouseMode::MouseMode_Line:
        type = EPathType::PathType_Line;
        break;
    case EMouseMode::MouseMode_Bezier:
        type = EPathType::PathType_Bezier;
        break;
    default:
        break;
    }
    return type;
}

void CMapGraphicsView::PaintMousePos(QPainter *painter, QPaintEvent *event)
{
    painter->setPen(QPen(QColor(0,0,0), 1));
    QRect rect = this->geometry();
    QStaticText textPosScene, textPosWorld;
    QPointF ptScene = mapToScene(m_ptCurrent.toPoint());
    double dWorldX,dWorldY;
    CMapManager::GetInstance()->MapToWorld(ptScene.x(), ptScene.y(), dWorldX, dWorldY);
    textPosScene.setText(QString("scene:%1,%2").arg(ptScene.x()).arg(ptScene.y()));
    textPosWorld.setText(QString("world:%1,%2").arg(dWorldX).arg(dWorldY));

    painter->setFont(QFont("楷体", 13, QFont::Normal));
    painter->drawStaticText(10, rect.height()-100, textPosScene);
    painter->drawStaticText(10, rect.height()-80, textPosWorld);
}

void CMapGraphicsView::PaintWorldAxis(QPainter *painter, QPaintEvent *event)
{
    double dMapZeroX, dMapZeroY;
    CMapManager::GetInstance()->WorldToMap(0, 0, dMapZeroX, dMapZeroY);
    QPointF pt = mapFromScene(dMapZeroX, dMapZeroY);// 场景转view
    painter->setPen(QPen(QColor(0,0,0), 1));

    //绘制X轴
    painter->drawLine(pt.x(), pt.y(), pt.x()+AXIS_LEN, pt.y());
    painter->drawLine(pt.x()+AXIS_LEN, pt.y(), pt.x()+AXIS_LEN-AXIS_POINTER_LEN, pt.y()-AXIS_POINTER_LEN);
    painter->drawLine(pt.x()+AXIS_LEN, pt.y(), pt.x()+AXIS_LEN-AXIS_POINTER_LEN, pt.y()+AXIS_POINTER_LEN);

    //绘制Y轴
    painter->drawLine(pt.x(), pt.y(), pt.x(), pt.y()+AXIS_LEN);
    painter->drawLine(pt.x(), pt.y()+AXIS_LEN, pt.x()+AXIS_POINTER_LEN, pt.y()+AXIS_LEN-AXIS_POINTER_LEN);
    painter->drawLine(pt.x(), pt.y()+AXIS_LEN, pt.x()-AXIS_POINTER_LEN, pt.y()+AXIS_LEN-AXIS_POINTER_LEN);

    //绘制文字x,y，（0，0）
    painter->setFont(QFont("楷体", 12, QFont::Normal));
    painter->drawText(pt+QPoint(-20,-10), "(0,0)");
    painter->drawText(QPointF(pt.x()+AXIS_LEN, pt.y())+QPointF(10,10), "x");
    painter->drawText(QPointF(pt.x(), pt.y()+AXIS_LEN)+QPointF(10,10), "y");

}

void CMapGraphicsView::PaintEraser(QPainter *painter, QPaintEvent *event)
{
    QPolygonF polygon;
    QPainterPath EraserPath;
    polygon << QPointF(m_ptCurrent.x()-m_iEraserSize/2, m_ptCurrent.y()-m_iEraserSize/2)
            << QPointF(m_ptCurrent.x()+m_iEraserSize/2, m_ptCurrent.y()-m_iEraserSize/2)
            << QPointF(m_ptCurrent.x()+m_iEraserSize/2, m_ptCurrent.y()+m_iEraserSize/2)
            << QPointF(m_ptCurrent.x()-m_iEraserSize/2, m_ptCurrent.y()+m_iEraserSize/2);
    EraserPath.addPolygon(polygon);
    m_pathEraser = EraserPath;
    m_pathEraser.setFillRule(Qt::WindingFill);
    //背景色
    double dOpacity = 0.5;//不透明度
    int ipenWidth = 1;
    Qt::PenStyle penStyle;
    painter->setRenderHints(QPainter::Antialiasing);//抗锯齿

    ipenWidth = 1;
    penStyle = Qt::PenStyle::DotLine;
    painter->setOpacity(dOpacity);
    painter->setBrush(QColor(219,112,147));

    //绘制轮廓
    painter->setPen(QPen(QColor(219,112,147), ipenWidth, penStyle));
    painter->drawPath(m_pathEraser);
}

void CMapGraphicsView::DeleteAGVItem()
{
    QList<QGraphicsItem *> listItems = items();
    for (auto &it : listItems)
    {
        if (it->type()==ITEM_AGV)
        {
            scene()->removeItem(it);
            delete it;
            m_pAGVItem = nullptr;
            break;
        }
    }
}

void CMapGraphicsView::DeleteAllItem()
{
    QList<QGraphicsItem *> listItems = items();
    for (auto &it : listItems)
    {
        if (it->type() != ITEM_MAP)
        {
            scene()->removeItem(it);
            delete it;
        }
    }
    m_pAGVItem = nullptr;
}

CStationItem *CMapGraphicsView::AddStationItem(TStation *ptstation)
{
    CStationItem *pStation = new CStationItem(ptstation->iStationID, this);
    //pos
    pStation->setPos(QPointF(ptstation->tPt.dX, ptstation->tPt.dY));
    //朝向角
    pStation->setTransform(pStation->transform().rotate(ptstation->dAngle-STATTION_INIT_ANGLE));
    //站点显示内容
    CStationTextItem *pStationText = new CStationTextItem(ptstation->iStationID);
    pStationText->MoveItemByStationItem(pStation->scenePos());
    pStation->SetStationTextItem(pStationText);
    scene()->addItem(pStation);
    scene()->addItem(pStationText);
    return pStation;
}

CLineItem* CMapGraphicsView::AddLineItem(int iPathID)
{
    CLineItem *pItem = new CLineItem(iPathID, this);
    scene()->addItem(pItem);
    return pItem;
}

void CMapGraphicsView::StationMove(int iStationID)
{
    //寻找连接的路线,修改它们的boundingrect，因为路线形状变了
    std::list<TPath *> listTpath = CMapManager::GetInstance()->GetStationLinkedPath(iStationID);
    QList<QGraphicsItem *> listItem = items();
    for (auto it : listItem)
    {
        if (it->type() == ITEM_LINE)
        {
            if(listTpath.size() >0)
            {
                std::list<TPath *>::iterator plist;
                for (plist = listTpath.begin(); plist != listTpath.end(); plist++)
                {
                    TPath * pPathTemp = *plist;
                    if(((CLineItem*)it)->m_iPathID == pPathTemp->iPathID)
                    {
                        ((CLineItem*)it)->StationMove();
                    }
                }
            }
            else
            {
                return;
            }
        }
    }
}

bool CMapGraphicsView::AddStationOnAGVPos()
{
    std::string strAGVID = CCoreControl::GetInstance()->GetCurrentMapAGVID();
    TAGVInfo *pAGV = CAGVManager::GetInstance()->GetAGV(strAGVID);
    if (pAGV->eStatus == EConnectStatus::Connected_Sucess)
    {
        QPointF ptSceneAGV;
        CMapManager::GetInstance()->WorldToMap(pAGV->tPt.dX, pAGV->tPt.dY, ptSceneAGV.rx(), ptSceneAGV.ry());
        CCmdBase *pCmd = new CCmdAddStation(this, ptSceneAGV, pAGV->dAngle, EStationType::StationType_General);
        CCommandManager::GetInstance()->AddCmd(pCmd);
        pCmd->exec();
        return true;
    }
    else
    {
        return false;
    }
}

bool CMapGraphicsView::MoveStationToAGV(TAGVInfo *pAGV)
{
    //查找选中的站点
    CStationItem *pItem = nullptr;
    QList<QGraphicsItem *> listItems = items();
    for (auto &it : listItems)
    {
        if (it->type()==ITEM_STATION && it->isSelected())
        {
            pItem = (CStationItem*)it;
            break;
        }
    }
    if (pItem == nullptr)
    {
        return false;
    }
    QPointF qptStBef, qptStAf,qptTxtBef, qptTxtAf;
    double dStAngleBef, dStAngleAf;
    auto itSt = CMapManager::GetInstance()->GetStation(pItem->GetStation());

    qptStBef = pItem->pos();
    qptTxtBef = pItem->GetTextItem()->pos();
    CMapManager::GetInstance()->WorldToMap(pAGV->tPt.dX, pAGV->tPt.dY, qptStAf.rx(), qptStAf.ry());
    qptTxtAf = qptTxtBef + qptStAf - qptStBef;

    dStAngleBef = itSt->dAngle;
    dStAngleAf = pAGV->dAngle;

    //移动了一个站点
    CCmdModifyStation *pCmd = new CCmdModifyStation(pItem, qptStBef, dStAngleBef, qptStAf, dStAngleAf,
                              qptTxtBef, qptTxtAf);
    CCommandManager::GetInstance()->AddCmd(pCmd);
    pCmd->exec();
    return true;
}

void CMapGraphicsView::ClearItemsSelectedStatus()
{
    m_listStationSelected.clear();
    for(auto &item : items())
    {
        item->setSelected(false);
    }
}

void CMapGraphicsView::AddGlobalPathItem(std::vector<TPointd> &vecPath)
{
    DeleGolbalPathItem();

    m_pGlobalItem = new CGlobalPathItem(this);
    m_pGlobalItem->SetPath(vecPath);

    scene()->addItem(m_pGlobalItem);
}

void CMapGraphicsView::DeleGolbalPathItem()
{
    if (m_pGlobalItem != nullptr)
    {
        scene()->removeItem(m_pGlobalItem);
        delete m_pGlobalItem;
        m_pGlobalItem = nullptr;
    }
}

void CMapGraphicsView::SlotrubberBandChanged(QRect viewportRect, QPointF fromScenePoint, QPointF toScenePoint)
{
    Q_UNUSED(viewportRect);
    Q_UNUSED(fromScenePoint);
    Q_UNUSED(toScenePoint);

    //m_bIsRuberSelecting = true; //不好用，这个函数会在鼠标抬起后再进入，没法弄了
}

void CMapGraphicsView::paintEvent(QPaintEvent *event)
{
    QGraphicsView::paintEvent(event);//放到最上面，这样先画scene中的Item

    QPainter painter(this->viewport());
    painter.setRenderHints(QPainter::Antialiasing);

    if (m_bIsDrawLine){
        painter.setPen(QPen(QColor(0, 0, 0), 1, Qt::PenStyle::DotLine));
        painter.drawLine(m_ptStartStation, m_ptCurrent);
        //qDebug() << m_ptStartStation << m_ptCurrent;
    }

    //绘场鼠标所在位置的场景坐标和世界坐标
    PaintMousePos(&painter, event);

    //绘制世界坐标轴
    PaintWorldAxis(&painter, event);

    //绘制橡皮擦//根据条件进行绘制
    if(m_eMouseMode==EMouseMode::MouseMode_Eraser && m_bIsEnterFlg)
    {
        PaintEraser(&painter, event);
    }

}

void CMapGraphicsView::contextMenuEvent(QContextMenuEvent *event)
{
//    QMenu menu;
//    menu.addAction("123");
//    menu.exec(event->globalPos());

    QGraphicsView::contextMenuEvent(event);
}

void CMapGraphicsView::enterEvent(QEvent *event)
{

    m_bIsEnterFlg = true;
}

void CMapGraphicsView::leaveEvent(QEvent *event)
{
    m_bIsEnterFlg = false;
    update();

}
