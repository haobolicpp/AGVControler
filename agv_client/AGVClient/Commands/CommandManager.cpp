#include <QDebug>
#include "CommandManager.h"
#include "CmdBase.h"
#include "CoreControl.h"

SINGLETON_IMPLEMENT(CCommandManager)

//指令队列上限
#define CMD_MAX 10

CCommandManager::CCommandManager()
{
    m_iPos = -1;
}

void CCommandManager::AddCmd(CCmdBase *pCmd)
{
    //将Pos后面的Cmd删除
    int iNum = m_listCmd.size()-m_iPos-1;
    for (int i=0; i<iNum; i++)
    {
        CCmdBase *pBase = m_listCmd.back();
        pBase->insertDelete();
        delete pBase;
        m_listCmd.pop_back();
    }

    //插入cmd
    m_listCmd.push_back(pCmd);

    //超出上限则删除头部数据
    if (m_listCmd.size() > CMD_MAX)
    {
        CCmdBase *pBase = m_listCmd.front();
        pBase->fullDelete();
        delete pBase;
        m_listCmd.pop_front();
    }
    else
    {
        m_iPos++;
    }

    CCoreControl::GetInstance()->SetCurrentMapModify(true);

    //qDebug() << "AddCmd" << m_iPos;
}

void CCommandManager::Clear()
{
    m_iPos = -1;
    while (!m_listCmd.empty())
    {
        CCmdBase *pCmd = m_listCmd.front();
        pCmd->fullDelete();
        pCmd->insertDelete();
        delete pCmd;
        m_listCmd.pop_front();
    }
}

bool CCommandManager::undo()
{
    if (m_iPos < 0)
    {
        return false;
    }
    int iTempPos = 0;
    for (auto cmd=m_listCmd.begin(); cmd!=m_listCmd.end(); cmd++)
    {
        if (iTempPos == m_iPos)
        {
            //pCommand->m_bIsExcute = GR_FALSE;
            (*cmd)->undo();
            m_iPos--;
            break;
        }
        iTempPos++;
    }

    //qDebug() << "undo" << m_iPos;

    CCoreControl::GetInstance()->SetCurrentMapModify(true);
    return true;
}

bool CCommandManager::redo()
{
    if (m_iPos >= (int)m_listCmd.size()-1)
    {
        return false;
    }

    m_iPos++;
    int iTempPos = 0;
    for (auto cmd=m_listCmd.begin(); cmd!=m_listCmd.end(); cmd++)
    {
        if (iTempPos == m_iPos)
        {
            //pCommand->m_bIsExcute = GR_TRUE;
            (*cmd)->exec();
            break;
        }
        iTempPos++;
    }

    //qDebug() << "redo" << m_iPos;
    CCoreControl::GetInstance()->SetCurrentMapModify(true);
    return true;
}
