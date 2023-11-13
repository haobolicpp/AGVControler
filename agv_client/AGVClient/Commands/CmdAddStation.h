#ifndef CCMDADDSTATION_H
#define CCMDADDSTATION_H
#include "CmdBase.h"
#include "MapView/StationItem.h"

class CMapGraphicsView;
class CCmdAddStation : public CCmdBase
{
public:
    CCmdAddStation(CMapGraphicsView *pView, QPointF pt, double dAngle, EStationType type);
    ~CCmdAddStation();

    /**
     * @brief exec
     */
    virtual void exec() override;

    /**
     * @brief undo
     */
    virtual void undo() override;

    /**
     * @brief fullDelete: 队列满调用
     */
    virtual void fullDelete() override;

    /**
     * @brief insertDelete : 撤销后新的操作，光标后面的删除
     */
    virtual void insertDelete() override;

private:
    //View指针
    CMapGraphicsView *m_pView;
    //站点添加的位置
    QPointF m_dPt;
    //站点朝向角
    double m_dAngle;
    //站点类型
    EStationType m_eType;
    //站点Item
    CStationItem *m_pStationItem;
    //站点对象
    TStation m_tStation;
};

#endif // CCMDADDSTATION_H
