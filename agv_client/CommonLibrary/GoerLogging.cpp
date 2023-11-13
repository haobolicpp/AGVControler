///******************************************************************************
//*
//*文件名称：QGoerLogging.cpp
//*摘    要：日志记录类实现
//*
//******************************************************************************/
//#include <QDebug>
//#include <QDateTime>
//#include <QFile>
//#include <QDir>
//#include "GoerLogging.h"
//#ifdef Q_OS_WIN
//#include <qt_windows.h>
//#endif
//#ifdef QT_USE_SLOG2
//#include <slog2.h>
//#endif
//#ifdef Q_OS_ANDROID
//#include <android/log.h>
//#endif

//#include "GoerToolUtile.h"
//#include "AutoIncBuffer.h"

//SINGLETON_IMPLEMENT(CGoerLogging)

//#define BUFFER_SIZE 1024*256 //256k

////临时存储缓冲区
//thread_local CAutoIncBuffer g_autoTempBuffer;

//CGoerLogging::CGoerLogging()
//{
//    m_bInit = GR_FALSE;
//    m_pthreadWrite = GR_NULL;
//    m_pWriteMtx = GR_NULL;
//    m_bRunning = GR_TRUE;
//    m_iLastTime = CGoerToolUtile::GetCurrentTimeSec();
//}

//CGoerLogging::~CGoerLogging()
//{
//    DeletePtr();
//}

////定时回调
//void TimerLoop(CGoerLogging *pLogging);

//GR_BOOL CGoerLogging::Init(QString qstrOutDirectory, GR_INT iMaxSize/*=10*/, GR_INT iWriteTime/*=30*/)
//{
//    return false;
//    m_iWriteMaxSz = iMaxSize << 20;
//    m_iWriteTime = iWriteTime;

//    do
//    {
//        m_pWriteMtx = new QMutex(QMutex::Recursive);
//        if (m_pWriteMtx == GR_NULL)
//        {
//            break;
//        }
//        m_pthreadWrite = new std::thread(TimerLoop, this);
//        if (GR_NULL == m_pthreadWrite)
//        {
//            break;
//        }

//        //目录创建
//        //If the directory already exists when this function is called, it will return false.
//        m_strOutDir = qstrOutDirectory;
//        GR_INT iPost = m_strOutDir.lastIndexOf('/');
//        if (iPost != m_strOutDir.length()-1)
//        {
//            m_strOutDir += "/";
//        }
//        QDir::current().mkdir(m_strOutDir);

//        //注册输出
//        //qInstallMessageHandler(MyMessageHandler);

//        m_bInit = GR_TRUE;
//        return GR_TRUE;
//    }
//    while(0);

//    DeletePtr();
//    return GR_FALSE;
//}

//void CGoerLogging::WriteImmediately()
//{
//    WriteOnce();
//}

//void CGoerLogging::GDebug(const char *pFormat, ...)
//{
//    Q_UNUSED(pFormat);
//#if defined(Debug) || defined(QT_DEBUG)
//    if (!m_bInit)
//    {
//        return;
//    }
//    va_list arglist;
//    va_start(arglist, pFormat);
//    InternalPrintf(Style_Debug, pFormat, arglist);
//    va_end(arglist);
//#endif
//}

//void CGoerLogging::GInfo(const char *pFormat, ...)
//{
//    if (!m_bInit)
//    {
//        return;
//    }
//    va_list arglist;
//    va_start(arglist, pFormat);
//    InternalPrintf(Style_Info, pFormat, arglist);
//    va_end(arglist);
//}

//void CGoerLogging::GWarning(const char *pFormat, ...)
//{
//    if (!m_bInit)
//    {
//        return;
//    }
//    va_list arglist;
//    va_start(arglist, pFormat);
//    InternalPrintf(Style_Warning, pFormat, arglist);
//    va_end(arglist);
//}

//void CGoerLogging::GError(const char *pFormat, ...)
//{
//    if (!m_bInit)
//    {
//        return;
//    }
//    va_list arglist;
//    va_start(arglist, pFormat);
//    InternalPrintf(Style_Error, pFormat, arglist);
//    va_end(arglist);
//}

//void CGoerLogging::InternalPrintf(EPrintfStyle eStyle, const char *pFormat, va_list pvalist)
//{
//    g_autoTempBuffer.AdjustSize(BUFFER_SIZE);//没法调用Init，这里特殊处理
//    GR_INT iRet = vsnprintf(g_autoTempBuffer.GetBuffer(), BUFFER_SIZE, pFormat, pvalist);
//    if (-1 == iRet)
//    {
//        return;
//    }

//    g_autoTempBuffer.GetBuffer()[BUFFER_SIZE-1] = '\0';

//    QDateTime qtmCurrent = QDateTime::currentDateTime();
//    QString qstrTm = qtmCurrent.toString("[yyyy-MM-dd-hh:mm:ss]");
//    QString qstrWrite;

//    QString qstrlogMessage = QString::fromLocal8Bit((GR_PCHAR)g_autoTempBuffer.GetBuffer());
//    qstrlogMessage.append(QLatin1Char('\n'));
//    qstrlogMessage = qstrTm + qstrlogMessage;

//    switch (eStyle)
//    {
//    case Style_Debug:
//        //只输出到控制台
//        qstrTm = "[Debug]";
//        break;
//    case Style_Info:
//        qstrTm = "[Info]:";
//        break;
//    case Style_Warning:
//        qstrTm = "[Warning]:";
//        break;
//    case Style_Error:
//        qstrTm = "[Error]:";
//        break;
//    default:
//        break;
//    }
//    qstrlogMessage = qstrTm + qstrlogMessage;

//    //输出到控制台
//    OutToConsole(qstrlogMessage);
//    qstrWrite += qstrlogMessage;
//    QMutexLocker locker(m_pWriteMtx);
//    m_strLog += qstrWrite;
//}

//void CGoerLogging::WriteOnce()
//{
//    QDateTime qtmCurrent = QDateTime::currentDateTime();

//    m_pWriteMtx->lock();
//    if (m_strLog.isEmpty())
//    {
//        m_pWriteMtx->unlock();
//        return;
//    }
//    m_pWriteMtx->unlock();

//    //创建日志文件名
//    if (m_strLastFileName.isEmpty())
//    {
//        m_strLastFileName = QString("log_%1.log").arg(QString("%1").arg(qtmCurrent.toString("yyyyMMddhhmmss")));
//    }
//    QFile file(m_strOutDir + m_strLastFileName);
//    if (file.exists())
//    {
//        if (file.size() > m_iWriteMaxSz)//文件过大
//        {
//            //重新创建新的文件
//            m_strLastFileName = QString("log_%1.log").arg(QString("%1").arg(qtmCurrent.toString("yyyyMMddhhmmss")));
//            file.setFileName(m_strOutDir + m_strLastFileName);
//        }
//    }

//    if (!file.open(QIODevice::ReadWrite | QIODevice::Text | QIODevice::Append))
//    {
//        return;
//    }

//    //printf("%s", m_strLog.toLocal8Bit().data());

//    QTextStream out(&file);

//    m_pWriteMtx->lock();
//    out << m_strLog;
//    m_strLog.clear();
//    m_pWriteMtx->unlock();
//    out.flush();

//}

////定时执行函数
//void TimerLoop(CGoerLogging *pLogging)
//{
//    while(pLogging->m_bRunning)
//    {
//        std::this_thread::sleep_for(std::chrono::seconds(1));//只休眠1s，让线程退出时不等待太久

//        GR_INT iCurrentTm = CGoerToolUtile::GetCurrentTimeSec();
//        if (iCurrentTm - pLogging->m_iLastTime >= pLogging->m_iWriteTime)
//        {
//            pLogging->WriteOnce();
//            pLogging->m_iLastTime = iCurrentTm;
//        }
//    }
//}

//void CGoerLogging::OutToConsole(QString strValue)
//{
//#if defined(Q_OS_WIN)
//     printf("%s", strValue.toStdString().c_str());
//     qDebug() << strValue;
//#elif defined(QT_USE_SLOG2)
//    slog2_default_handler(type, logMessage.toLocal8Bit().constData());
//#elif defined(QT_USE_JOURNALD) && !defined(QT_BOOTSTRAPPED)
//    systemd_default_message_handler(type, context, logMessage);
//#elif defined(QT_USE_SYSLOG) && !defined(QT_BOOTSTRAPPED)
//    syslog_default_message_handler(type, logMessage.toUtf8().constData());
//#elif defined(Q_OS_ANDROID)
//    android_default_message_handler(type, context, logMessage);
//#endif
//#ifdef Q_OS_LINUX
//    printf("%s", strValue.toLocal8Bit().data());
//#endif
//}

//void CGoerLogging::Destory()
//{
//    m_bRunning = GR_FALSE;
//    m_pthreadWrite->join();
//    DeletePtr();
//}

//void CGoerLogging::DeletePtr()
//{
//    DELETE_PTR(m_pthreadWrite);
//    DELETE_PTR(m_pWriteMtx);
//}
