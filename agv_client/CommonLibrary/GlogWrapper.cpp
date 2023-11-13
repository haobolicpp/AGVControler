#include <QDir>
#include "GlogWrapper.h"

SINGLETON_IMPLEMENT(CGlogWrapper)

CGlogWrapper::CGlogWrapper()
{

}

bool CGlogWrapper::Init(std::string strLogDir)
{
    //创建日志目录
    QDir dir;
    if(!dir.mkpath(strLogDir.c_str()))
    {
        return false;
    }
    // Initialize Google’s logging library.
    google::InitGoogleLogging("");
    //禁止清除日志
    google::DisableLogCleaner();
    //日志目录
    FLAGS_log_dir = strLogDir.c_str();

    return true;
}
