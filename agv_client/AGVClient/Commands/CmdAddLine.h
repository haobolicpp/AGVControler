#ifndef CCMDADDLINE_H
#define CCMDADDLINE_H
#include "CmdBase.h"
#include "AGVClient_def.h"
#include "MapView/LineItem.h"

class CCmdAddLine : public CCmdBase
{
public:
    CCmdAddLine(CMapGraphicsView *pView, EPathType eType, int iStartStation, int iEndStation);
    ~CCmdAddLine();

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
    // 线
    CLineItem *m_pLineItem;
    int m_iPathID;

    CMapGraphicsView *m_pView;
    EPathType m_eType;
    int m_iStartStation;
    int m_iEndStation;

};

#endif // CCMDADDLINE_H
