#ifndef CCONFIGMANAGER_H
#define CCONFIGMANAGER_H
#include "commonlibrary_global.h"
#include "SingletonMacro_Def.h"
#include <unordered_map>
#include <QString>
#include <string>

class COMMONLIBRARYSHARED_EXPORT CConfigManager
{
    SINGLETON_DECLARE(CConfigManager)
public:
    CConfigManager();

    //初始化配置文件
    bool InitConfigFile(QString strConfigFile);

    //查询配置
    QString GetValueString(QString strKey, bool *bOK=nullptr);
    int GetValueInt(QString strKey, bool *bOK=nullptr);
    double GetValueDouble(QString strKey, bool *bOK=nullptr);

private:
    std::unordered_map<std::string, QString> m_mapValue;
};

#endif // CCONFIGMANAGER_H
