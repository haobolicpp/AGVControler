/******************************************************************************
*
*文件名称: WaitingWidget.h
*摘    要: 等待中UI，会进行蒙版设置
*
******************************************************************************/
#ifndef CWAITINGWIDGET_H
#define CWAITINGWIDGET_H
#include <QMovie>
#include <QTimer>
#include <QWidget>
#include <QLabel>
#include <functional>
#include "AGVClientUIEx_global.h"

class CWaitingWidget;
class AGVCLIENTUIEX_EXPORT CWaitingWidgetControl : public QObject
{
    Q_OBJECT
public:
    explicit CWaitingWidgetControl(QWidget *parent = nullptr);

    /**
     * @brief Show : 弹窗显示等待窗口
     * @param strText : 显示的文本
     * @param iWaitingTime : 等待超时的时间，ms，0为无穷等
     * @param funcClose : 等待超时时间到后回调函数，0无效
     */
    void Show(QString strText, int iWaitingTime=0, std::function<void(void)> funcTimeout=nullptr);
    void Close();

public slots:
    void SlotOnTimer();

private:
    QMovie *m_pmv;
    QTimer *m_ptimer;
    CWaitingWidget *m_pDlg;
    std::function<void(void)> m_funcTimeout;
    //显示gif图片label
    QLabel *m_plabelGIF;
    //显示文本的label
    QLabel *m_plabelTxt;
    //显示的文本
    QString m_strTxt;
    //定时计数
    int m_iTick;
    //超时时长
    int m_iTimeout;
};

#endif // CWAITINGWIDGET_H
