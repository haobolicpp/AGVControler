#include <QFile>
#include <QDir>
#include <QFileInfo>
#include <QCryptographicHash>
#include "json.hpp"
#include "AGVManager.h"
#include "MapManager.h"
#include "GoerToolUtile.h"
#include "JsonUtil.h"
#include "CoreControl.h"

SINGLETON_IMPLEMENT(CAGVManager)

using json = nlohmann::json;

CAGVManager::CAGVManager()
{
    //测试数据
//    for (int i=0; i<30; i++)
//    {
//        TAGVInfo tAGVInfo;
//        tAGVInfo.strAGVID = std::to_string(i+1);
//        tAGVInfo.eStatus = EConnectStatus::Connected_Invalid;
//        m_mapAGVs.insert(make_pair(tAGVInfo.strAGVID, tAGVInfo));
//    }
}

bool CAGVManager::Init()
{
    //枚举所有的AGV目录
    QString strExeDir = CGoerToolUtile::GetExeDirectory();
    QString strAGVsDir = strExeDir + "/AGVs";
    QDir dir(strAGVsDir);
    if (!dir.exists())
    {
        return false;
    }
    dir.setFilter(QDir::Dirs | QDir::NoDotAndDotDot);
    dir.setSorting(QDir::Name);
    QStringList listDir = dir.entryList(); //目录（agv编号列表）
    for (int i = 0; i < listDir.size(); ++i)
    {
        auto itAGV = m_mapAGVs.insert(make_pair(listDir.at(i).toStdString(), TAGVInfo{}));
        itAGV.first->second.strAGVID = itAGV.first->first;

        //读取AGV基础信息
        QString strAGVInfoDir = strAGVsDir + "/" + listDir.at(i) + "/agv_info.json";
        QFile fileAGVInfo(strAGVInfoDir);
        if (!fileAGVInfo.open(QIODevice::ReadOnly))
        {
            return false;
        }
        QByteArray buffAGVInfo = fileAGVInfo.readAll();
        fileAGVInfo.close();
        if (!json::accept(buffAGVInfo.data()))
        {
            return false;
        }
        auto itAGVInfo = json::parse(buffAGVInfo.data());
        itAGV.first->second.strAGVIP = itAGVInfo.value("ip", "111.111.111.111");
        itAGV.first->second.strMapName = itAGVInfo.value("map_name", "default");
        itAGV.first->second.strMapMD5 = itAGVInfo.value("map_md5", "");
        itAGV.first->second.strComment = itAGVInfo.value("comment", "");
        itAGV.first->second.bChecked = false;
    }

    return true;
}

TAGVInfo *CAGVManager::GetAGV(std::string strAGVID)
{
    auto itAGV = m_mapAGVs.find(strAGVID);
    if (itAGV == m_mapAGVs.end()){
        return nullptr;
    }else{
        return &itAGV->second;
    }
}

TAGVInfo *CAGVManager::GetCurrentAGV()
{
    return GetAGV(CCoreControl::GetInstance()->GetCurrentMapAGVID());
}

void CAGVManager::DeleteAGV(string strAGVID)
{
    //删除内存信息
    auto itAGV = m_mapAGVs.find(strAGVID);
    if (itAGV != m_mapAGVs.end())
    {
        m_mapAGVs.erase(itAGV);
    }

    //删除文件
    QString strExeDir = CGoerToolUtile::GetExeDirectory();
    QString strAGVsDir = strExeDir + "/AGVs/";
    QDir qDir(strAGVsDir+strAGVID.c_str());
    qDir.removeRecursively();
}

const std::unordered_map<string, TAGVInfo> &CAGVManager::GetAllAGV()
{
    return m_mapAGVs;
}

int CAGVManager::AGVLogin(string strAGVID, string strIP, bool bSucess, TBroadcastResData* pData)
{
    //更新lambda
    auto itUpdateFunc = [&](TAGVInfo *tInfo){
        tInfo->strAGVID = strAGVID;
        tInfo->strAGVIP = strIP;
        if (bSucess){
            tInfo->eStatus = EConnectStatus::Connected_Sucess;
        }else{
            tInfo->eStatus = EConnectStatus::Connected_False;
        }
        tInfo->strVersion = to_string(pData->uVersion);
        //其他信息初始化
        UpdateAGVInfo(strAGVID, pData->tRTData);
    };

    auto itAGV = m_mapAGVs.find(strAGVID);
    if (itAGV == m_mapAGVs.end())
    {
        TAGVInfo tAGVInfo;
        m_mapAGVs[strAGVID] = tAGVInfo;
        itUpdateFunc(&tAGVInfo);
        //创建本地AGV配置文件
        return CreateAGVConfig(&m_mapAGVs[strAGVID]);
    }
    else
    {
        itUpdateFunc(&itAGV->second);
    }
}

void CAGVManager::UpdateAGVInfo(std::string strAGVID, TAGVRealTimeData tData)
{
    auto itAGV = m_mapAGVs.find(strAGVID);
    if (itAGV != m_mapAGVs.end())
    {
        itAGV->second.tPt.dX = tData.fAGVX;
        itAGV->second.tPt.dY = tData.fAGVY;
        itAGV->second.iCPU = tData.chCPU;
        itAGV->second.iRAM = tData.chRAM;
        itAGV->second.dAngle = CGeometryAlgorithm::RadToDeg(tData.fAngle);
        itAGV->second.dLineSpeed = tData.fLinearSpeed;
        itAGV->second.dConfidence = tData.fCondfidence;
        itAGV->second.dAngularSpeed = tData.fAngularSpeed;
        itAGV->second.iElectricity = tData.chElec;
        itAGV->second.eAGVRunMode = (EAGVRunMode)tData.chAGVRunMode;
        itAGV->second.strControllerMapName = tData.chMapName;
        itAGV->second.strControllerMapMD5 = tData.chMapMD5;

        if (itAGV->second.strControllerMapName != itAGV->second.strMapName
                || itAGV->second.strMapMD5 != itAGV->second.strControllerMapMD5)
        {
            itAGV->second.bIsSync = false;
        }
        else
        {
            itAGV->second.bIsSync = true;
        }
    }
    else
    {
        assert(false);
    }
}

void CAGVManager::AGVDisconnect(string strAGVID)
{
    auto itAGV = m_mapAGVs.find(strAGVID);
    if (itAGV != m_mapAGVs.end())
    {
        itAGV->second.eStatus = EConnectStatus::Connected_Invalid;
    }
    else
    {
        assert(false);
    }
}

int CAGVManager::SaveAGVBaseInfo(TAGVInfo *pAGVInfo)
{
    QString strExeDir = CGoerToolUtile::GetExeDirectory();
    QString strAGVsDir = strExeDir + "/AGVs";
    QString strAGVInfoDir = strAGVsDir + "/" + pAGVInfo->strAGVID.c_str() + "/agv_info.json";

    json jvalue;
    jvalue["ip"] = pAGVInfo->strAGVIP;
    jvalue["map_name"] = pAGVInfo->strMapName;
    jvalue["map_md5"] = pAGVInfo->strMapMD5;
    jvalue["comment"] = pAGVInfo->strComment;
    std::string strJson = jvalue.dump(0);

    QFile file(strAGVInfoDir);
    if (!file.open(QIODevice::WriteOnly))
    {
        return (int)EAGVManagerErr::FileNotExist;
    }
    file.write(strJson.c_str());
    file.flush();
    file.close();
    return (int)EAGVManagerErr::Sucess;
}

int CAGVManager::SaveAGVConfigInfo(TAGVInfo *pAGVInfo)
{
    return (int)EAGVManagerErr::Sucess;
}

int CAGVManager::CreateAGVConfig(TAGVInfo *pAGVInfo)
{
    QString strAGVsDir = CGoerToolUtile::GetExeDirectory() + "/AGVs";
    QString strAGVMapsDir = strAGVsDir + "/" + pAGVInfo->strAGVID.c_str() + "/maps";
    //创建目录
    QDir dir;
    if(!dir.mkpath(strAGVMapsDir))
    {
        return (int)EAGVManagerErr::CreateFileFailed;
    }

    //保存基础信息
    int iret = SaveAGVBaseInfo(pAGVInfo);
    if (iret != (int)EAGVManagerErr::Sucess){return iret;}

    //写setting.json
    iret = SaveAGVConfigInfo(pAGVInfo);
    if (iret != (int)EAGVManagerErr::Sucess){return iret;}

    return (int)EAGVManagerErr::Sucess;
}
