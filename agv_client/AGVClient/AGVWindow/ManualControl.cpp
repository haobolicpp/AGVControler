#include <QDebug>
#include <QMessageBox>
#include "ManualControl.h"
#include "ui_ManualControl.h"
#include "NetManager.h"
#include "CoreControl.h"
#include "WaitingWidgetControl.h"

CManualControlWidget::CManualControlWidget(QWidget *parent) :
    CWidgetBase(nullptr),
    ui(new Ui::CManualControl)
{
    ui->setupUi(this);
    this->setFixedSize(this->width(), this->height());

    connect(ui->btn_up, &QPushButton::pressed, this, [&](){BtnPressedPri((QPushButton *)sender(), true);});
    connect(ui->btn_down, &QPushButton::pressed, this, [&](){BtnPressedPri((QPushButton *)sender(), true);});
    connect(ui->btn_left, &QPushButton::pressed, this, [&](){BtnPressedPri((QPushButton *)sender(), true);});
    connect(ui->btn_right, &QPushButton::pressed, this, [&](){BtnPressedPri((QPushButton *)sender(), true);});

    connect(ui->btn_up, &QPushButton::released, this, [&](){BtnPressedPri((QPushButton *)sender(), false);});
    connect(ui->btn_down, &QPushButton::released, this, [&](){BtnPressedPri((QPushButton *)sender(), false);});
    connect(ui->btn_left, &QPushButton::released, this, [&](){BtnPressedPri((QPushButton *)sender(), false);});
    connect(ui->btn_right, &QPushButton::released, this, [&](){BtnPressedPri((QPushButton *)sender(), false);});

    m_iBtnData = 0;

    m_ptimerSend = new QTimer(this);
    connect(m_ptimerSend, &QTimer::timeout, this, [&](){
        CNetManager::GetInstance()->SendMoveCtrl(CCoreControl::GetInstance()->GetCurrentMapAGVID(), m_iBtnData);
        //qDebug() << m_iBtnData ;
    });

    m_pWaitingWdgt = new CWaitingWidgetControl(parent);

    REGISTER_CALLBACK(Class_Control, Type_Control_Teleoperation, this->OnTeleoperation);

}

void CManualControlWidget::ShowCtrl()
{
    m_bPorcCallback = true;
    if (CNetManager::GetInstance()->SendTeleoperation(CCoreControl::GetInstance()->GetCurrentMapAGVID(), 1))
    {
        m_pWaitingWdgt->Show("等待控制器应答", 5000, [&](){
            m_bPorcCallback = false;
            hide();
            m_ptimerSend->stop();
            QMessageBox::information(this, "", "等待超时！");
        });
    }
}

void CManualControlWidget::HideCtrl()
{
    m_bPorcCallback = true;
    if (CNetManager::GetInstance()->SendTeleoperation(CCoreControl::GetInstance()->GetCurrentMapAGVID(), 0))
    {
        m_pWaitingWdgt->Show("等待控制器应答", 5000, [&](){
            m_bPorcCallback = false;
            hide();
            m_ptimerSend->stop();
            QMessageBox::information(this, "", "等待超时！");
        });
    }
    else
    {
        m_bPorcCallback = false;
        hide();
        m_ptimerSend->stop();
    }
}

CManualControlWidget::~CManualControlWidget()
{
    delete ui;
}

void CManualControlWidget::keyPressEvent(QKeyEvent *event)
{
    if(event->key()==Qt::Key_Left || event->key()==Qt::Key_A)    //左
    {
        BtnPressedPri(ui->btn_left, true);
    }
    else if(event->key()==Qt::Key_Right || event->key()==Qt::Key_D)    //右
    {
        BtnPressedPri(ui->btn_right, true);
    }
    else if(event->key()==Qt::Key_Up || event->key()==Qt::Key_W)    //上
    {
        BtnPressedPri(ui->btn_up, true);
    }
    else if(event->key()==Qt::Key_Down || event->key()==Qt::Key_S)    //下
    {
        BtnPressedPri(ui->btn_down, true);
    }
    else{}
}

void CManualControlWidget::keyReleaseEvent(QKeyEvent *event)
{
    if(event->key()==Qt::Key_Left || event->key()==Qt::Key_A)    //左
    {
        BtnPressedPri(ui->btn_left, false);
    }
    else if(event->key()==Qt::Key_Right || event->key()==Qt::Key_D)    //右
    {
        BtnPressedPri(ui->btn_right, false);
    }
    else if(event->key()==Qt::Key_Up || event->key()==Qt::Key_W)    //上
    {
        BtnPressedPri(ui->btn_up, false);
    }
    else if(event->key()==Qt::Key_Down || event->key()==Qt::Key_S)    //下
    {
        BtnPressedPri(ui->btn_down, false);
    }
    else{}
}

void CManualControlWidget::closeEvent(QCloseEvent *event)
{
    HideCtrl();
    event->ignore();
}

void CManualControlWidget::BtnPressedPri(QPushButton *pBtn, bool bPressed)
{
    int ioffset = 0;

    if (pBtn == ui->btn_up)
    {
        ioffset = 0;
    }
    else if (pBtn == ui->btn_down)
    {
        ioffset = 1;
    }
    else if (pBtn == ui->btn_left)
    {
        ioffset = 2;
    }
    else
    {
        ioffset = 3;
    }

    if (bPressed)
    {
        m_iBtnData |= (0x01 << ioffset);
    }
    else
    {
        m_iBtnData &= ~(0x01 << ioffset);
    }

}

void CManualControlWidget::OnTeleoperation(TCallBackData *pData)
{
    if (!m_bPorcCallback)
    {
        return;
    }

    m_pWaitingWdgt->Close();

    int iRet = *((int*)pData->pData);
    if (iRet == -1)
    {
        hide();
        m_ptimerSend->stop();
        QMessageBox::information(this, "", "控制器返回失败，无法进行遥操作！");
    }
    else if (iRet == 0)
    {
        hide();
        m_ptimerSend->stop();
    }
    else if (iRet == 1)
    {
        show();
        m_ptimerSend->start(50);
    }
    else
    {
        hide();
        m_ptimerSend->stop();
        QMessageBox::information(this, "", "控制器返回未知错误，无法进行遥操作！");
    }

    //free (pData->pData);
}
