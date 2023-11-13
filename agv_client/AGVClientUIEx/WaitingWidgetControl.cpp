#include <QVBoxLayout>
#include "WaitingWidget.h"
#include "WaitingWidgetControl.h"

#define TICK_TIME 1000

CWaitingWidgetControl::CWaitingWidgetControl(QWidget *parent)
{
    m_pmv = new QMovie(parent);
    m_ptimer = new QTimer(parent);
    m_pDlg = new CWaitingWidget(parent);

    m_pmv->setFileName(":/Image/resource/waiting.gif");
    m_pmv->start();

    m_plabelGIF = new QLabel(parent);
    m_plabelTxt = new QLabel(parent);
    m_plabelGIF->setMovie(m_pmv);
    m_plabelTxt->setFont(QFont("SimHei", 0, QFont::Bold));

    QVBoxLayout *pVLayout = new QVBoxLayout;//垂直
    pVLayout->addWidget(m_plabelGIF);
    pVLayout->addWidget(m_plabelTxt);
    pVLayout->setAlignment(m_plabelTxt,Qt::AlignCenter);

    m_pDlg->setWindowLayout(pVLayout);

    connect(m_ptimer, &QTimer::timeout, this, &CWaitingWidgetControl::SlotOnTimer);

    m_pDlg->hide();
    m_iTick = 0;
    m_iTimeout = 0;
}

void CWaitingWidgetControl::Show(QString strText, int iWaitingTime, std::function<void(void)> funcTimeout)
{
    m_strTxt = strText;
    m_plabelTxt->setText(m_strTxt);
    m_funcTimeout = funcTimeout;
    m_pDlg->show();
    m_iTimeout = iWaitingTime;
    m_ptimer->start(TICK_TIME);
}

void CWaitingWidgetControl::Close()
{
    m_ptimer->stop();
    m_pDlg->hide();
    m_iTimeout = 0;
}

void CWaitingWidgetControl::SlotOnTimer()
{
    m_iTick++;

    //动态显示点
    int iDotNum = m_iTick%3+1;
    QString strTemp;
    for (int i=0; i<iDotNum; i++){
        strTemp += ".";
    }
    m_plabelTxt->setText(m_strTxt+strTemp);

    //判断是否超时
    if (m_iTick*TICK_TIME > m_iTimeout)
    {
        if (m_funcTimeout){
            m_funcTimeout();
            Close();
            m_iTick = 0;
        }
    }
}
