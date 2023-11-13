#include "CNameHelper.h"

CNameHelper::CNameHelper()
{

}

QString CNameHelper::TransAGVConnectStatus(EConnectStatus eStatus)
{
    if (eStatus == EConnectStatus::Connected_Invalid)
    {
        return QStringLiteral("未连接");
    }
    else if (eStatus == EConnectStatus::Connected_False)
    {
        return QStringLiteral("连接失败");
    }
    else if (eStatus == EConnectStatus::Connected_Sucess)
    {
        return QStringLiteral("连接成功");
    }
    else{}
    return QString();
}

QString CNameHelper::TransSync(bool bisSync)
{
    if (bisSync)
    {
        return QStringLiteral("已同步");
    }
    else
    {
        return QStringLiteral("未同步");
    }
}
