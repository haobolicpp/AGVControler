#ifndef CCMDBASE_H
#define CCMDBASE_H
#include "AGVClient_def.h"

class CCmdBase
{
public:
    CCmdBase();
    virtual ~CCmdBase();

    /**
     * @brief exec
     */
    virtual void exec() = 0;

    /**
     * @brief undo
     */
    virtual void undo() = 0;

    /**
     * @brief fullDelete: 队列满调用
     */
    virtual void fullDelete() = 0;

    /**
     * @brief insertDelete : 撤销后新的操作，光标后面的删除
     */
    virtual void insertDelete() = 0;

    friend class CCommandManager;
};

#endif // CCMDBASE_H
