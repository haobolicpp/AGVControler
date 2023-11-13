///******************************************************************************
//*
//*文件名称：GoerLogging.h
//*摘    要：日志记录类声明。

//>提供【调试Debug】、【信息Info】、【警告Warning】、【错误Error】四种等级。
//>>>>>>>【调试Debug】的不会写入到文件中<<<<<<<<
//>Qt系统的qDebug等接口不会写入文件（为了防止系统调用影响文件内容）
//>写入文件的，默认都添加时间戳。
//>外部可设置日志文件的大小，超过大小会重新创建新的文件，默认文件名为log_年月日时分秒。
//>为了提升性能，定时往文件中写,可设置定时时间。

//---------------该类已废弃--------------------
//---------------该类已废弃--------------------
//---------------该类已废弃--------------------
//---------------该类已废弃--------------------
//*
//*
//******************************************************************************/
//#pragma once
//#include <QMutex>
//#include <QString>
//#include <QMutex>
//#include <thread>
//#include <stdarg.h>
//#include "SingletonMacro_Def.h"
//#include "gr_type.h"
//#include "commonlibrary_global.h"

////格式串类型
//enum EPrintfStyle
//{
//    Style_Debug,
//    Style_Info,
//    Style_Warning,
//    Style_Error,
//};

////使用相关宏定义,简化调用代码
//#define LOG_WARN CGoerLogging::GetInstance()->GWarning
//#define LOG_INFO CGoerLogging::GetInstance()->GInfo
//#define LOG_ERROR CGoerLogging::GetInstance()->GError
//#define LOG_DEBUG CGoerLogging::GetInstance()->GDebug

////带文件+行号的打印
////C++11要求，当字符串跟变量连接的时候，必须增加一个空格才行
//#define LOG_FL_WARN(format, ...) CGoerLogging::GetInstance()->GWarning("file:%s,line:%i," format, __FILE__, __LINE__, ##__VA_ARGS__);
//#define LOG_FL_INFO(format, ...) CGoerLogging::GetInstance()->GInfo("file:%s,line:%i," format, __FILE__, __LINE__, ##__VA_ARGS__);
//#define LOG_FL_ERROR(format, ...) CGoerLogging::GetInstance()->GError("file:%s,line:%i," format, __FILE__, __LINE__, ##__VA_ARGS__);
//#define LOG_FL_DEBUG(format, ...) CGoerLogging::GetInstance()->GDebug("file:%s,line:%i," format, __FILE__, __LINE__, ##__VA_ARGS__);

//class COMMONLIBRARYSHARED_EXPORT CGoerLogging
//{
//    friend void MyMessageHandler(QtMsgType, const QMessageLogContext &, const QString &);

//    SINGLETON_DECLARE(CGoerLogging)
//public:
//    CGoerLogging();
//    ~CGoerLogging();

//    /******************************************************************************
//    * 函数名称: Init
//    * 功能描述: 初始化模块
//    * 输入参数: QString qstrOutDirectory：日志输出目录
//                GR_INT iMaxSize=10:log文件大小，单位M，默认10M
//                GR_INT iWriteTime=30：定时写时间，单位秒，默认30s
//    * 输出参数: 无
//    * 返 回 值: GR_TRUE成功
//    ******************************************************************************/
//    GR_BOOL Init(QString qstrOutDirectory,
//                 GR_INT iMaxSize=10,
//                 GR_INT iWriteTime=30);

//    //立即写入日志（调用后立即将内存中的数据写入日志）
//    void WriteImmediately();

//    //调用接口
//    void GDebug(const char *pFormat, ...);
//    void GInfo(const char *pFormat, ...);
//    void GWarning(const char *pFormat, ...);
//    void GError(const char *pFormat, ...);

//private:
//    //内部格式串
//    void InternalPrintf(EPrintfStyle eStyle, const char *pFormat, va_list pvalist);
//    //定时回调
//    friend void TimerLoop(CGoerLogging *pLogging);
//    //写入一次数据
//    void WriteOnce();
//    //输出到控制台
//    void OutToConsole(QString strValue);
//    //删除内存数据
//    void DeletePtr();

//private:
//    QString m_strOutDir;        //输出目录
//    QString m_strLog;           //日志内容,可优化为std::string，因为QString调用clear后会free掉内存
//    QMutex *m_pWriteMtx;        //立即写锁
//    QString m_strLastFileName;  //上次写入的日志文件名称
//    GR_INT m_iWriteMaxSz;       //日志文件的最大大小,单位字节
//    GR_BOOL m_bInit;            //该模块是否初始化
//    GR_INT m_iWriteTime;        //定时写入时间
//    std::thread *m_pthreadWrite;//后台写入线程
//    volatile GR_BOOL m_bRunning; //线程退出标志
//    GR_INT m_iLastTime;        // 线程计时(s)
//};

