
#ifndef CGLOGWRAPPER_H
#define CGLOGWRAPPER_H
#include "commonlibrary_global.h"
#include "glog/logging.h"
#include "SingletonMacro_Def.h"
#include <QDebug>

/**
 * @brief The CGlogWrapper class
 * 打印格式：
 * I20210729 10:51:11.655335  6748 GlogWrapper.cpp:26] file
 * I是INFO,后面是年月日 时分秒   线程ID 文件         行号   内容
 *
 * 用法：
 * GLOG_INFO << "内容" << "其他"
 */

#ifdef QT_DEBUG
#define GLOG_INFO qDebug()
#define GLOG_WARN qDebug()
#define GLOG_ERR qDebug()
#define GLOG_FATAL qDebug()
#else
#define GLOG_INFO LOG(INFO)
#define GLOG_WARN LOG(WARNING)
#define GLOG_ERR LOG(ERROR)
#define GLOG_FATAL LOG(FATAL)
#endif

class COMMONLIBRARYSHARED_EXPORT CGlogWrapper
{
    SINGLETON_DECLARE(CGlogWrapper)
public:
    CGlogWrapper();

    /**
     * @brief Init : 初始化
     * @param strLogDir : 日志的目录
     * @return
     */
    bool Init(std::string strLogDir);
};

#endif // CGLOGWRAPPER_H
