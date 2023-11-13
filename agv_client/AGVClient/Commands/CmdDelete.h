#ifndef CCMDDELETE_H
#define CCMDDELETE_H
#include <list>
#include <set>
#include "CmdBase.h"
#include "MapView/StationItem.h"
#include "MapView/LineItem.h"
#include "MapView/MapGraphicsView.h"

typedef struct TStationData
{
    CStationItem *pItem;
    TStation tStation;
}TStationData;

typedef struct TLineData
{
    CLineItem *pItem;
    TPath tPath;
}TLineData;

class CCmdDelete : public CCmdBase
{
public:
    CCmdDelete(CMapGraphicsView *pView, std::set<CStationItem*> &setStaion, std::set<CLineItem*> &setLine);

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
    std::list<TStationData> m_listStaion;
    std::list<TLineData> m_listLine;
    CMapGraphicsView *m_pView;
};

#endif // CCMDDELETE_H
