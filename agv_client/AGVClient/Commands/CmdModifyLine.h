#ifndef CCMDMODIFYLINE_H
#define CCMDMODIFYLINE_H
#include "CmdBase.h"

class CCmdModifyLine : public CCmdBase
{
public:
    CCmdModifyLine();

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
};

#endif // CCMDMODIFYLINE_H
