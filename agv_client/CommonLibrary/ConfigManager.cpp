#include "ConfigManager.h"
#include <qjsondocument.h>
#include <QJsonParseError>
#include <qjsonobject.h>
#include <qjsonarray.h>
#include <stdio.h>
#include <QFile>
#include <QDebug>

SINGLETON_IMPLEMENT(CConfigManager)

CConfigManager::CConfigManager()
{
}

bool CConfigManager::InitConfigFile(QString strConfigFile)
{
    //读取json格式的配置文件
    QFile qFile(strConfigFile);
    qFile.open(QIODevice::ReadOnly);
    QByteArray qb = qFile.readAll();
    qFile.close();

    //解析json
    QJsonParseError jpe;
    QJsonDocument qjd(QJsonDocument::fromJson(qb, &jpe));
    if (jpe.error != QJsonParseError::NoError)
    {
        return false;
    }

   QJsonObject obj = qjd.object();
   QStringList qlst = obj.keys();
   for (auto &it : qlst){
       if (!obj.value(it).isString()){
           return false;
       }
       m_mapValue.insert(make_pair(it.toStdString(), obj.value(it).toString()));
   }
   return true;
}

QString CConfigManager::GetValueString(QString strKey, bool *bOK)
{
    auto it = m_mapValue.find(strKey.toStdString());
    if (it != m_mapValue.end()){
        if (bOK){
            *bOK = true;
        }
        return it->second;
    }

    if (bOK){
        *bOK = false;
    }
    return "";
}

int CConfigManager::GetValueInt(QString strKey, bool *bOK)
{
    auto it = m_mapValue.find(strKey.toStdString());
    if (it != m_mapValue.end()){
        if (bOK){
            *bOK = true;
        }
        return it->second.toInt();
    }
    if (bOK){
        *bOK = false;
    }
    return -1;
}

double CConfigManager::GetValueDouble(QString strKey, bool *bOK)
{
    auto it = m_mapValue.find(strKey.toStdString());
    if (it != m_mapValue.end()){
        if (bOK){
            *bOK = true;
        }
        return it->second.toDouble();
    }
    if (bOK){
        *bOK = false;
    }
    return -1;
}

