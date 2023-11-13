#ifndef CCMDMODIFYSTATION_H
#define CCMDMODIFYSTATION_H
#include "CmdBase.h"
#include "MapView/StationItem.h"

class CCmdModifyStation : public CCmdBase
{
public:
    CCmdModifyStation(CStationItem *pItem, QPointF tLastPt, double dLastAngle,
                      QPointF tNowPt, double dNowAngle, QPointF lastTxtPt, QPointF nowTxtPt);

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
    CStationItem *m_pStationItem;

    //站点上次的值
    QPointF m_dLasttPt; //当前坐标，scene像素坐标，注意(0,0)位置
    double m_lastdAngle; //朝向角，°范围(0~180 0~-180)

    //站点新值
    QPointF m_dNowtPt; //当前坐标，scene像素坐标，注意(0,0)位置
    double m_nowdAngle; //朝向角，°范围(0~180 0~-180)

    //站点文本上次位置
    QPointF m_tlastTxt;
    //站点文本新位置
    QPointF m_tnowTxt;
};

#endif // CCMDMODIFYSTATION_H
