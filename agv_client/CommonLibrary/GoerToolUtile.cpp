/******************************************************************************

*
*文件名称：GoerToolUtile.cpp
*摘要：工具类，提供一些通用的接口
*
*
******************************************************************************/
#include "GoerToolUtile.h"
#include <QCoreApplication>
#include <QList>
#include <QDateTime>
#include <QDir>

CGoerToolUtile::CGoerToolUtile(void)
{

}

CGoerToolUtile::~CGoerToolUtile()
{

}

QStringList CGoerToolUtile::GetFileNameFromDirectory(QString strDirectory, QStringList strNameFilters)
{
    QStringList strNameFiltersInter;
    QDir dir;

    if (strDirectory.isEmpty())
    {
        return QStringList();
    }

    if (strNameFilters.isEmpty())
    {
        strNameFiltersInter << "*.*";
    }
    else
    {
        strNameFiltersInter = strNameFilters;
    }

    dir.setPath(strDirectory);
    dir.setNameFilters(strNameFiltersInter);
    dir.setFilter(QDir::Files);

    return dir.entryList();
}

QString CGoerToolUtile::GetExeDirectory()
{
    return QCoreApplication::applicationDirPath();
}

GR_BOOL CGoerToolUtile::GetExeUpDirectory(QString & strDir, GR_INT iForntNum)
{
    strDir = QCoreApplication::applicationDirPath();
    GR_INT iPos = 0;
    for (GR_INT i = 1; i <= iForntNum;i++)
    {
        iPos = strDir.lastIndexOf('/');
        if (iPos == -1)
        {
            return GR_FALSE;
        }
        strDir = strDir.left(iPos); //路径上升一�?
    }
    return GR_TRUE ;
}

GR_S64 CGoerToolUtile::GetCurrentTimeMSec()
{
    return (GR_S64)QDateTime::currentMSecsSinceEpoch();
}

QString CGoerToolUtile::HexToString(char *pbuffer, int ilen)
{
    QString strRet;
    for (int i=0; i<ilen; i++)
    {
        strRet += " " + QString::asprintf("%02x", pbuffer[i]).right(2);
    }
    return strRet;
}

GR_INT CGoerToolUtile::GetCurrentTimeSec()
{
    return (GR_INT)(QDateTime::currentMSecsSinceEpoch()/1000);
}
