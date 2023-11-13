#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTableWidgetItem>
#include <QStackedWidget>
#include <QBoxLayout>
#include <QLabel>
#include "AGVListWindow/AGVListWindow.h"
#include "AGVWindow/AGVWidget.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    //切换到AGVWindow窗口
    void ChangeToAGVWindow();
    void ChangeToHome();
    //AGV掉线
    void AGVDisconnect(std::string strAGVID);
    //更新AGV实时数据信息
    void UpdateAGVStatusbar();
protected:
    virtual void closeEvent(QCloseEvent *event) override;
    //重置状态栏信息
    void ResetStatusBar();

private:
    Ui::MainWindow *ui;
    QStackedWidget *m_pstackWdgt;
    QGridLayout *m_pLayout;
    CAGVListWindow *m_plistWdgt;
    CAGVWidget *m_pagvWdgt;

    //状态栏控件
    QLabel m_labAGVID;
    QLabel m_labAGVIP;
    QLabel m_labAGVConnectStatus;
    QLabel m_labElectricity;  //电量 %
    QLabel m_labConfidence;  //置信度 0~1
    QLabel m_labIsSync;  //地图及数据是否是同步的
    QLabel m_labSpeed; //[线速度m/s,角速度 rad/s]
    QLabel m_labtPos; //当前位姿[x,y,车头朝向(°)]


    friend class CAGVListWindow;
};

#endif // MAINWINDOW_H
