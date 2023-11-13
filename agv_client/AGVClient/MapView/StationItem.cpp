/*
 * 站点Item的中心点在scene中会被移动，和LineItem不一样。
*/
#include <QPainter>
#include <QStyleOptionGraphicsItem>
#include <QPolygonF>
#include <QDebug>
#include <QGraphicsSceneHoverEvent>
#include "StationItem.h"
#include "MapView_def.h"
#include "GeometryAlgorithm.h"
#include "MapView/StationTextItem.h"
#include "MapGraphicsView.h"
#include "Commands/CmdModifyStation.h"
#include "Commands/CommandManager.h"
#include "MapManager.h"

CStationItem::CStationItem(int iStationID, CMapGraphicsView *pView, QGraphicsItem *parent):
    QGraphicsObject(parent)
{
    m_pView = pView;
    m_iStationID = iStationID;
    m_bIsHoverEnter = false;
    m_bIsPressInCrontrolPoint = false;
    m_bIsPressInItem = false;

    //轮廓形状，按车头朝向STATTION_INIT_ANGLE(-90)绘制
    QPolygonF polygon;
    polygon << QPointF(-STATION_WIDTH/2, -STATION_HIGHT/2)
            << QPointF(STATION_WIDTH/2, -STATION_HIGHT/2)
            << QPointF(STATION_WIDTH/2, STATION_HIGHT/2)
            << QPointF(-STATION_WIDTH/2, STATION_HIGHT/2)
            << QPointF(-STATION_WIDTH/2, -STATION_HIGHT/2);
    m_borderPath.addPolygon(polygon);
    QPainterPath linePath;
    polygon.clear();
    polygon << QPointF(0,-STATION_HIGHT/2) << QPointF(STATION_WIDTH/4, -STATION_WIDTH/4)
            << QPointF(-STATION_WIDTH/4, -STATION_WIDTH/4) << QPointF(0,-STATION_HIGHT/2);
    m_borderPath.addPolygon(polygon);

    //控制点
    m_controlPointPath.addEllipse(QRectF(-STATION_CONTROL_LEN, -STATION_HIGHT/2-STATION_CONTROL_LEN,
                                         STATION_CONTROL_LEN*2, STATION_CONTROL_LEN*2));

    m_allPath.addPath(m_borderPath);
    m_allPath.addPath(m_controlPointPath);

    //标记
    setFlags(QGraphicsItem::ItemIsMovable | QGraphicsItem::ItemIsSelectable
             |QGraphicsItem::ItemSendsGeometryChanges);
    setAcceptHoverEvents(true);//可激发hoverXXXEvent回调
}

void CStationItem::MouseMove(QPointF pos)
{
    if (m_bIsPressInCrontrolPoint) return; //调整旋转过程，直接返回

    pos = mapFromScene(pos);
    if (m_controlPointPath.contains(pos)){
        setCursor(QCursor(Qt::CursorShape::CrossCursor));
    }else{
        setCursor(QCursor(Qt::CursorShape::SizeAllCursor));
    }
    //Item外面移动时会自动动恢复成指针形状
}

QRectF CStationItem::boundingRect() const
{
    return m_allPath.boundingRect();
}

//只进入时执行一次
void CStationItem::hoverEnterEvent(QGraphicsSceneHoverEvent *event)
{
    if (m_pView->GetMouseMode() != EMouseMode::MouseMode_Pointer)
    {
        return;
    }
    QGraphicsItem::hoverEnterEvent(event);
    m_bIsHoverEnter = true;
}

void CStationItem::hoverLeaveEvent(QGraphicsSceneHoverEvent *event)
{
    if (m_pView->GetMouseMode() != EMouseMode::MouseMode_Pointer)
    {
        return;
    }
    QGraphicsItem::hoverLeaveEvent(event);
    m_bIsHoverEnter = false;
    update();
}

void CStationItem::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
    emit m_pView->SignalStationPress(m_iStationID);

    if (m_controlPointPath.contains(event->pos())){
        m_bIsPressInCrontrolPoint = true;
        setFlag(QGraphicsItem::ItemIsMovable, false);
        m_ptCurrent = event->pos();
        m_matrix = matrix();
    }
    else
    {
        m_bIsPressInItem = true;

        //重置上一次的位置，方便连续移动多个站点
        if (!m_pView->m_listStationSelected.empty())
        {
            for (auto &item : m_pView->m_listStationSelected)
            {
                item.tLastStationPt = item.pItem->pos();
                item.tLastStationTxtPt = item.pItem->m_pTextItem->pos();
            }
        }
    }
    QGraphicsObject::mousePressEvent(event);
    update();
    m_tLastTxtPos = m_pTextItem->pos();
    m_tLastStationPos = pos();
    auto itStation = CMapManager::GetInstance()->GetStation(m_iStationID);
    m_dLastAngle = itStation->dAngle;
}

//只在按下鼠标时执行，在boundingrect中。
void CStationItem::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{
    if (!m_bIsPressInCrontrolPoint){
        //更新站点位置
        auto itStation = CMapManager::GetInstance()->GetStation(m_iStationID);
        itStation->tPt.dX = pos().x();
        itStation->tPt.dY = pos().y();
        QGraphicsObject::mouseMoveEvent(event);

        //移动绑定的TextItem
        m_pTextItem->MoveItemByStationItem(scenePos());

        //更新其他选中站点的站点和TextItem位置
        for (auto item : m_pView->m_listStationSelected)
        {
            if (item.pItem != this)
            {
                //其他站点位置数据更新
                auto itStation2 = CMapManager::GetInstance()->GetStation(item.pItem->GetStation());
                if(itStation2 == nullptr)
                {
                    assert(false);
                    return;
                }
                itStation2->tPt.dX = item.pItem->pos().x();
                itStation2->tPt.dY = item.pItem->pos().y();
                item.pItem->GetTextItem()->MoveItemByStationItem(item.pItem->scenePos());
            }
        }
        return;
    }

    m_ptCurrent = event->pos();
    setCursor(QCursor(Qt::CursorShape::CrossCursor));

    //进行Item旋转控制
    //【原理】旋转的是当前坐标系，一次鼠标事件来临，deg仅会有微小变化，rotate会矫正它和-90初始朝向的误差
    QTransform trans = transform();
    double deg = CGeometryAlgorithm::Point2PointAngle(0,0, event->pos().x(), event->pos().y()); //计算当前点和中心原点的夹角
    double degdiff = deg - (-90); //初始-90
    //qDebug() << "deg:" << deg << "degdiff:" << degdiff << "m_ptStation->dAngle"<<m_ptStation->dAngle << event->pos();
    trans.rotate(degdiff); //顺时针旋转坐标系，因为y轴向下，所以顺时针旋转
    setTransform(trans);

    QGraphicsObject::mouseMoveEvent(event);
}


void CStationItem::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
    auto itStation = CMapManager::GetInstance()->GetStation(m_iStationID);
    if (m_bIsPressInCrontrolPoint){
        m_bIsPressInCrontrolPoint = false;
        setFlag(QGraphicsItem::ItemIsMovable, true);
        //计算朝向角，在Item中无法计算，因为此时当前鼠标点和中心点夹角还是-90,需要转换到scene中计算
        QPointF ptZero = mapToScene(QPointF{0,0});
        QPointF ptMouse = mapToScene(event->pos());
        double deg = CGeometryAlgorithm::Point2PointAngle(ptZero.x(),ptZero.y(), ptMouse.x(), ptMouse.y());
        itStation->dAngle = deg;
        //qDebug() << deg;

        //添加旋转站点的修改命令
        CCommandManager::GetInstance()->AddCmd(new CCmdModifyStation(
                              this, m_tLastStationPos, m_dLastAngle, this->pos(), itStation->dAngle,
                              m_tLastTxtPos, m_pTextItem->pos()));
    }
    else
    {
        //记录当前pos
//        QPointF pos = this->pos();
//        itStation->tPt.dX = pos.x();
//        itStation->tPt.dY = pos.y();
        //通知移动完成事件
        m_pView->StationMove(m_iStationID);

        if (m_bIsPressInItem)
        {
            //判断是否批量移动了站点
            if (m_pView->m_listStationSelected.empty())
            {
                //移动了一个站点
                CCommandManager::GetInstance()->AddCmd(new CCmdModifyStation(
                                      this, m_tLastStationPos, m_dLastAngle, this->pos(), itStation->dAngle,
                                      m_tLastTxtPos, m_pTextItem->pos()));
            }
            else
            {
                //移动了一堆站点
                for (auto &station : m_pView->m_listStationSelected)
                {
                    station.tNowStationPt = station.pItem->scenePos();
                    station.tNowStationTxtPt = station.pItem->GetTextItem()->scenePos();
                    //路线形状改变
                    m_pView->StationMove(station.pItem->GetStation());
                }
                //批量移动指令
                CCommandManager::GetInstance()->AddCmd(new CCmdMoveStations(m_pView->m_listStationSelected));


            }
        }
    }
    m_bIsPressInItem = false;
    QGraphicsObject::mouseReleaseEvent(event);
    update();
    emit m_pView->SignalStationModify(m_iStationID);

}

void CStationItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    QColor colorBackGround;
    int ipenWidth = 1;
    Qt::PenStyle penStyle;
    painter->setRenderHints(QPainter::Antialiasing);//抗锯齿
    auto itStation = CMapManager::GetInstance()->GetStation(m_iStationID);

    //背景色
    double dOpacity = 1.0;//不透明度

    if ((option->state & QStyle::State_Selected) || m_bIsHoverEnter){
        if (itStation->eType == EStationType::StationType_General){
            colorBackGround = GENERAL_COLOR_SELECTED;
        }else{
            colorBackGround = CHARGE_COLOR_SELECTED;
        }
    }else{
        if (itStation->eType == EStationType::StationType_General){
            colorBackGround = GENERAL_COLOR;
        }else{
            colorBackGround = CHARGE_COLOR;
        }
        dOpacity = 0.7;
    }
    ipenWidth = 1;
    penStyle = Qt::PenStyle::SolidLine;
    painter->setOpacity(dOpacity);
    painter->setBrush(colorBackGround);

    //绘制轮廓
    painter->setPen(QPen(QColor(0, 0, 0), ipenWidth, penStyle));
    painter->drawPath(m_borderPath);
    //painter->drawLine(0,0,100,100);

    //控制点
    if ((option->state & QStyle::State_Selected) || m_bIsHoverEnter){
        painter->fillPath(m_controlPointPath, QBrush(QGradient(QGradient::CrystalRiver)));
    }

    //控制点拖动下，鼠标点和中心原点连线
    if (m_bIsPressInCrontrolPoint){
        painter->setPen(QPen(QColor(25, 25, 112), 0.5, Qt::PenStyle::DotLine));
        painter->drawLine(0, 0, m_ptCurrent.x(), m_ptCurrent.y());
        QPainterPath path;
        path.addEllipse(QRectF(m_ptCurrent.x()-STATION_CONTROL_LEN, m_ptCurrent.y()-STATION_CONTROL_LEN,
                               STATION_CONTROL_LEN*2, STATION_CONTROL_LEN*2));
        painter->fillPath(path, QBrush(QGradient(QGradient::CrystalRiver)));
    }
}


