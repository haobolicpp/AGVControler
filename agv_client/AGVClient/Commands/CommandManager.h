#ifndef CCOMMANDMANAGER_H
#define CCOMMANDMANAGER_H
#include <list>
#include "SingletonMacro_Def.h"

class CCmdBase;
class CCommandManager
{
    SINGLETON_DECLARE(CCommandManager)
public:
    CCommandManager();

    /**
     * @brief AddCmd
     * @param pCmd
     */
    void AddCmd(CCmdBase *pCmd);

    /**
     * @brief Clear: 清空命令信息
     */
    void Clear();

    /**
     * @brief undo
     * @return false表示无法再撤销了
     */
    bool undo();

    /**
     * @brief redo
     * * @return false表示无法再回退了
     */
    bool redo();

private:
    std::list<CCmdBase*> m_listCmd;
    //当前是哪个CMD的下标
    int m_iPos;
};

#endif // CCOMMANDMANAGER_H
