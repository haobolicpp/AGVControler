#include <QMessageBox>
#include <QHeaderView>
#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "HeaderViewEx.h"
#include "AGVClient_def.h"
#include "AGVManager.h"
#include "MapManager.h"
#include "CNameHelper.h"
#include "CoreControl.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    //setWindowState(Qt::WindowMaximized);

    m_pstackWdgt = new QStackedWidget(this);
    m_pLayout = new QGridLayout(this);
    m_plistWdgt = new CAGVListWindow(this);
    m_pagvWdgt = new CAGVWidget(this);
    m_pstackWdgt->addWidget(m_plistWdgt);
    m_pstackWdgt->addWidget(m_pagvWdgt);
    m_pstackWdgt->setCurrentIndex(0);
    m_pLayout->addWidget(m_pstackWdgt);
    ui->centralWidget->setLayout(m_pLayout);

    ResetStatusBar();

    //状态栏
    ui->statusBar->hide();
    ui->statusBar->addWidget(&m_labAGVID);
    ui->statusBar->addWidget(&m_labAGVIP);
    ui->statusBar->addWidget(&m_labAGVConnectStatus);
    ui->statusBar->addWidget(&m_labElectricity);
    ui->statusBar->addWidget(&m_labConfidence);
    ui->statusBar->addWidget(&m_labIsSync);
    ui->statusBar->addWidget(&m_labSpeed);
    ui->statusBar->addWidget(&m_labtPos);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::ChangeToAGVWindow()
{
    m_pagvWdgt->InitData();
    m_pstackWdgt->setCurrentIndex(1);
    ui->statusBar->show();
}

void MainWindow::ChangeToHome()
{
    m_pstackWdgt->setCurrentIndex(0);
    ui->statusBar->hide();
}

void MainWindow::AGVDisconnect(string strAGVID)
{
    std::string strAGVIDMap = CCoreControl::GetInstance()->GetCurrentMapAGVID();
    if (strAGVID == strAGVIDMap)
    {
        ResetStatusBar();
        m_pagvWdgt->AGVDisconnect();
    }
}

void MainWindow::UpdateAGVStatusbar()
{
    if (m_pstackWdgt->currentIndex() == 0)
    {
        return;
    }

    std::string strAGVID = CCoreControl::GetInstance()->GetCurrentMapAGVID();
    TAGVInfo *pAGVInfo = CAGVManager::GetInstance()->GetAGV(strAGVID);
    m_labAGVID.setText(strAGVID.c_str());
    m_labAGVIP.setText(pAGVInfo->strAGVIP.c_str());
    m_labAGVConnectStatus.setText(CNameHelper::TransAGVConnectStatus(pAGVInfo->eStatus));
    m_labElectricity.setText(QString("电量:%1").arg(pAGVInfo->iElectricity));
    m_labConfidence.setText(QString::asprintf("置信度:%.2f", pAGVInfo->dConfidence));
    m_labIsSync.setText(CNameHelper::TransSync(pAGVInfo->bIsSync));
    m_labSpeed.setText(QString::asprintf("V[%.2fm/s, %.2frad/s]", pAGVInfo->dLineSpeed, pAGVInfo->dAngularSpeed));
    m_labtPos.setText(QString::asprintf("P[%.2fm, %.2fm, %.2f°]", pAGVInfo->tPt.dX, pAGVInfo->tPt.dY, pAGVInfo->dAngle));

}

void MainWindow::closeEvent(QCloseEvent *event)
{
    //网络通信停止
    CNetManager::GetInstance()->Close();
    QMainWindow::closeEvent(event);
}

void MainWindow::ResetStatusBar()
{
    m_labAGVID.setText(QStringLiteral("AGVID"));
    m_labAGVIP.setText(QStringLiteral("AGVIP"));
    m_labAGVConnectStatus.setText(QStringLiteral("连接状态"));
    m_labElectricity.setText(("电量"));
    m_labConfidence.setText(("置信度"));
    m_labIsSync.setText(("是否同步"));
    m_labSpeed.setText(("V[速度信息]"));
    m_labtPos.setText(("P[位姿信息]"));
}
