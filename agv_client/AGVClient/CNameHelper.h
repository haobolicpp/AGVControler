#ifndef NAMEHELPER_H
#define NAMEHELPER_H
#include <QString>
#include "AGVClient_def.h"

class CNameHelper
{
public:
    CNameHelper();

    /**
     * @brief TransAGVConnectStatus 状态转换
     * @param eStatus
     * @return
     */
    static QString TransAGVConnectStatus(EConnectStatus eStatus);

    /**
     * @brief TransSync 是否同步转换
     * @param bisSync
     * @return
     */
    static QString TransSync(bool bisSync);
};

#endif // NAMEHELPER_H
