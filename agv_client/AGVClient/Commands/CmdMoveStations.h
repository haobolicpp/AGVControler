/*
 * 批量移动站点
*/
#ifndef CMOVESTATIONS_H
#define CMOVESTATIONS_H
#include <list>
#include "CmdBase.h"
#include "MapView/StationItem.h"

//移动的站点数据
typedef struct TMoveStationsData
{
    CStationItem *pItem;
    QPointF tLastStationPt;
    QPointF tNowStationPt;
    QPointF tLastStationTxtPt;
    QPointF tNowStationTxtPt;
}TMoveStationsData;

class CCmdMoveStations : public CCmdBase
{
public:
    CCmdMoveStations(std::list<TMoveStationsData> listStationItem);

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
    std::list<TMoveStationsData> m_listMoveStations;
};

#endif // CMOVESTATIONS_H
