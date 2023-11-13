#include <QApplication>
#include <QPushButton>
#include <QFile>
#include <QSharedPointer>
#include "mainwindow.h"
#include "GoerToolUtile.h"
#include "ConfigManager.h"
#include "mainwindow.h"
#include "NetManager.h"
#include "LockFreeRingBuffer.h"
#include "AGVManager.h"
#include "MapManager.h"
#include "GlogWrapper.h"

//注册自定义类型
//Q_DECLARE_METATYPE(TPluginData)
//Q_DECLARE_METATYPE(TImageMatEx)
void RegisterSelfType()
{
//    qRegisterMetaType<TPluginData>("TPluginData");
//    qRegisterMetaType<TImageMatEx>("TImageMatEx");
}

void SetGloablQss()
{
    QString strEXEUpPath = CGoerToolUtile::GetExeDirectory();
    strEXEUpPath += "\\Resources\\project.qss";
    QFile qss(strEXEUpPath);
    qss.open(QFile::ReadOnly);
    qApp->setStyleSheet(qss.readAll());
    qss.close();
};

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    a.setWindowIcon(QIcon(QString(":/Image/agv_client.ico")));

    if (!CGlogWrapper::GetInstance()->Init("d:/agv_client_log"))
    {
        return false;
    }

    GLOG_INFO << "程序启动";

    //设置全局样式表
    SetGloablQss();

    //注册自定义类型
    RegisterSelfType();

    //配置文件加载
    QString strExePath = CGoerToolUtile::GetExeDirectory();
    CConfigManager::GetInstance()->InitConfigFile(strExePath+"/Config/sys.json");

    int a1 = CConfigManager::GetInstance()->GetValueInt("key1");
    QString str1 = CConfigManager::GetInstance()->GetValueString("key2");

    //AGV管理模块初始化
     CAGVManager::GetInstance()->Init();

    //网络模块初始化
    if (!CNetManager::GetInstance()->Init()){
        GLOG_ERR << "CNetManager error";
        return 0;
    }

    //系统core配置参数初始化，必须在相机初始化之前
    //if (CSysConfigManager::GetInstance()->Init() != 0)
    //{
    //    CGoerMessageBox::GoerMsgBoxError(QStringLiteral("系统参数读取失败！"));
    //    CGoerLogging::GetInstance()->GError((GR_PCHAR)"系统参数读取失败！");
    //    CGoerLogging::GetInstance()->WriteImmediately();
    //    return 0;
    //}

    MainWindow mw;
    mw.setWindowTitle("AGV控制器配置软件");
    mw.showMaximized();

    return a.exec();

}
