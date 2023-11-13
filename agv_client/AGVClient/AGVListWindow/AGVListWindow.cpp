#include <QMessageBox>
#include <QScrollBar>
#include "AGVListWindow.h"
#include "ui_AGVListWindow.h"
#include "HeaderViewEx.h"
#include "AGVClient_def.h"
#include "mainwindow.h"
#include "NetManager.h"
#include "agv_msg_def.h"
#include "AGVManager.h"
#include "MapManager.h"
#include "AGVWindow/AGVWidget.h"
#include "CNameHelper.h"
#include "CoreControl.h"

enum class EAGVIndex{
    AGV_Check,
    AGV_IP,
    AGV_ConnectStatus,
    AGV_Electric,
    AGV_Confidence,
    AGV_SyncStatus,
    AGV_Mapname,
    AGV_ID,
    AGV_Comment,
    AGV_Max
};

CAGVListWindow::CAGVListWindow(MainWindow *pwnd, QWidget *parent) :
    CWidgetBase(parent),
    ui(new Ui::CAGVListWindow)
{
    ui->setupUi(this);
    m_pMainWnd = pwnd;

    //agv列表初始化
    CHeaderViewEx *phvex = new CHeaderViewEx(Qt::Horizontal, this); //这里必须是this，如果是table控件不行的
    QStringList strList;
    strList <<  QStringLiteral("全选") << "IP" << QStringLiteral("连接状态") << QStringLiteral("电量") << QStringLiteral("置信度")
            << QStringLiteral("同步状态") << QStringLiteral("地图名称") << "ID" << QStringLiteral("备注");
    ui->tableAGVs->setHorizontalHeader(phvex);
    ui->tableAGVs->verticalHeader()->setVisible(false);
    ui->tableAGVs->setEditTriggers(QAbstractItemView::NoEditTriggers);  //禁用编辑
    ui->tableAGVs->setSelectionBehavior(QAbstractItemView::SelectRows);  //整行选中
    ui->tableAGVs->setColumnCount(strList.length());
    ui->tableAGVs->setHorizontalHeaderLabels(strList);
    ui->tableAGVs->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch); //表格自适应宽度
    ui->tableAGVs->verticalScrollBar()->setVisible(true); //垂直滚动条可见
    connect(phvex, &CHeaderViewEx::SigCheckboxClick, [&](Qt::CheckState stat){
        for (int i=0; i<ui->tableAGVs->rowCount(); i++)
        {
            QTableWidgetItem *pItem = ui->tableAGVs->item(i, (int)EAGVIndex::AGV_Check);
            pItem->setCheckState(stat); //全选或反选
        }
    });

    //WiFi列表初始化
    QHeaderView *ph = new QHeaderView(Qt::Horizontal, this); //这里必须是this，如果是table控件不行的
    strList.clear();
    strList <<  QStringLiteral("名称") << QStringLiteral("连接状态");
    ui->tableWidget_WIFI->setHorizontalHeader(ph);
    ui->tableWidget_WIFI->verticalHeader()->setVisible(false);
    ui->tableWidget_WIFI->setEditTriggers(QAbstractItemView::NoEditTriggers);  //禁用编辑
    ui->tableWidget_WIFI->setSelectionBehavior(QAbstractItemView::SelectRows);  //整行选中
    ui->tableWidget_WIFI->setColumnCount(strList.length());
    ui->tableWidget_WIFI->setHorizontalHeaderLabels(strList);
    ui->tableWidget_WIFI->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch); //表格自适应宽度

    //插入测试数据
    UpdateAGVList();
    ui->tableWidget_WIFI->setRowCount(10);
    ui->tableWidget_WIFI->setItem(0, 0, new QTableWidgetItem("Goertek"));
    ui->tableWidget_WIFI->setItem(0, 1, new QTableWidgetItem("已连接"));
    ui->tableWidget_WIFI->setItem(1, 0, new QTableWidgetItem("Test"));
    ui->tableWidget_WIFI->setItem(1, 1, new QTableWidgetItem(""));

    //注册连接监听
    connect(this, &CAGVListWindow::SignalAGVConnect, this, &CAGVListWindow::SlotRecvAGVConnectInfo);
    CNetManager::GetInstance()->RegisterConnect([&](std::string strAGVID, std::string strAGVIP, bool bSucess, TBroadcastResData tData){
        TBroadcastResData *pData = new TBroadcastResData(tData);
        emit SignalAGVConnect(strAGVID.c_str(), strAGVIP.c_str(), bSucess, pData);
    });

    //注册断线监听
    connect(this, &CAGVListWindow::SignalAGVDisconnect, this, &CAGVListWindow::SlotRecvAGVDisConnectInfo);
    CNetManager::GetInstance()->RegisterDisconnect([&](std::string strAGVID){
        emit SignalAGVDisconnect(strAGVID.c_str());
    });

    //注册AGV实时数据信息
    REGISTER_CALLBACK(Class_Control, Type_Control_RealTimeData, this->RecvAGVReportData);

    //启动刷新定时器
    m_timerUpdateAGVList.setParent(this);
    connect(&m_timerUpdateAGVList, &QTimer::timeout, [&](){UpdateAGVList();});
    m_timerUpdateAGVList.start(1000);
}

CAGVListWindow::~CAGVListWindow()
{
    delete ui;
}

void CAGVListWindow::on_tableAGVs_itemDoubleClicked(QTableWidgetItem *item)
{
    //获取AGVID
    QTableWidgetItem *pAGVItem = ui->tableAGVs->item(item->row(), (int)EAGVIndex::AGV_ID);
    if (pAGVItem != nullptr)
    {
        std::string strAGVID = pAGVItem->text().toStdString();
        TAGVInfo *pAGV = CAGVManager::GetInstance()->GetAGV(strAGVID);
        //读取AGV的地图信息
        int iret = CMapManager::GetInstance()->ReadMapJsonFromAGVDir(pAGV);
        if (iret == (int)EAGVManagerErr::FileNotExist)
        {
            CMapManager::GetInstance()->InitDefaultMap();
            QMessageBox::warning(this, "", QString("地图%1,不存在！").arg(pAGV->strMapName.c_str()));
        }
        else if (iret == (int)EAGVManagerErr::JsonError)
        {
            QMessageBox::warning(this, "", QString("地图%1,格式错误！").arg(pAGV->strMapName.c_str()));
            return;
        }else{}
        //标记当前选中的AGV
        CCoreControl::GetInstance()->SetCurrentMapAGVID(strAGVID);
        //初始化数据并跳转页面
        m_pMainWnd->ChangeToAGVWindow();
    }
    else
    {
        QMessageBox::warning(this, "", "no agv");
    }

}

void CAGVListWindow::on_pushButton_ScanAGV_clicked()
{
    CNetManager::GetInstance()->SendBroadcast();
}

void CAGVListWindow::SlotRecvAGVConnectInfo(QString strAGVID, QString strAGVIP, QVariant bSucess, void *pData)
{
    TBroadcastResData *pResData = (TBroadcastResData*)pData;

    //插入AGV数据
    CAGVManager::GetInstance()->AGVLogin(strAGVID.toStdString(), strAGVIP.toStdString(), bSucess.toBool(), pResData);

    delete pResData;
}

void CAGVListWindow::SlotRecvAGVDisConnectInfo(QString strAGVID)
{
    m_pMainWnd->AGVDisconnect(strAGVID.toStdString());
    CAGVManager::GetInstance()->AGVDisconnect(strAGVID.toStdString());
}

void CAGVListWindow::RecvAGVReportData(TCallBackData *pData)
{
    TAGVRealTimeData *pRealData = (TAGVRealTimeData*)pData->pData;

    //发送应答消息
    //CNetManager::GetInstance()->SendRealTimeDataRes(std::to_string(pData->tHeader.shSrc));

    //更新AGV数据
    CAGVManager::GetInstance()->UpdateAGVInfo(std::to_string(pData->tHeader.shSrc), *pRealData);

    //通知AGV展示页面刷新
    m_pMainWnd->m_pagvWdgt->m_pMapView->OnUpdateAGVRealTimeData();

    //刷新状态栏AGV数据
    m_pMainWnd->UpdateAGVStatusbar();

    //delete pRealData;
}

void CAGVListWindow::on_pushButton_Delete_clicked()
{
    QMessageBox::StandardButton btn =
            QMessageBox::information(this, "", "是否删除指定的AGV?",
                          QMessageBox::StandardButton::Ok, QMessageBox::StandardButton::Cancel);
    if (btn == QMessageBox::StandardButton::Cancel)
    {
        return;
    }


    //只允许删除已经断线的AGV。
    bool bHaveOnline = false;
    int iCount = ui->tableAGVs->rowCount();
    for (int i=iCount-1; i>=0; i--) //从后往前删
    {
        QTableWidgetItem *pItem = ui->tableAGVs->item(i, (int)EAGVIndex::AGV_Check);
        if (pItem->checkState() == Qt::CheckState::Checked)
        {
            QTableWidgetItem *pAGVItem = ui->tableAGVs->item(pItem->row(), (int)EAGVIndex::AGV_ID);
            if (pAGVItem != nullptr)
            {
                std::string strAGVID = pAGVItem->text().toStdString();
                TAGVInfo *pAGV = CAGVManager::GetInstance()->GetAGV(strAGVID);
                if (pAGV->eStatus == EConnectStatus::Connected_Sucess)
                {
                    bHaveOnline = true;
                }
                else
                {
                    //删除本AGV内存信息和配置文件信息
                    CAGVManager::GetInstance()->DeleteAGV(strAGVID);
                    //删除表格中的数据
                    ui->tableAGVs->removeRow(i);
                }
            }
        }
    }
    if(bHaveOnline)
    {
        QMessageBox::information(this, "", "在线的AGV无法删除！");
    }
}

void CAGVListWindow::UpdateAGVList()
{
    const std::unordered_map<string, TAGVInfo> &mapAGVs = CAGVManager::GetInstance()->GetAllAGV();
    ui->tableAGVs->setRowCount(mapAGVs.size());
    int irow = 0;
    QString strTemp;
    for(auto &itAGV : mapAGVs)
    {
        //check状态
        QTableWidgetItem *pLastItem = ui->tableAGVs->item(irow, (int)EAGVIndex::AGV_Check);
        if (pLastItem == nullptr)
        {
            QTableWidgetItem *pCheck = new QTableWidgetItem;
            pCheck->setCheckState(Qt::CheckState::Unchecked);
            ui->tableAGVs->setItem(irow, (int)EAGVIndex::AGV_Check, pCheck);
        }

        //地图名称
        ui->tableAGVs->setItem(irow, (int)EAGVIndex::AGV_Mapname, new QTableWidgetItem(itAGV.second.strMapName.c_str()));
        //AGVID
        ui->tableAGVs->setItem(irow, (int)EAGVIndex::AGV_ID, new QTableWidgetItem(itAGV.second.strAGVID.c_str()));
        //注释
        ui->tableAGVs->setItem(irow, (int)EAGVIndex::AGV_Comment, new QTableWidgetItem(itAGV.second.strComment.c_str()));
        //连接状态
        ui->tableAGVs->setItem(irow, (int)EAGVIndex::AGV_ConnectStatus, new QTableWidgetItem(CNameHelper::TransAGVConnectStatus(itAGV.second.eStatus)));

        if (itAGV.second.eStatus == EConnectStatus::Connected_Invalid)
        {
            //IP
            ui->tableAGVs->setItem(irow, (int)EAGVIndex::AGV_IP, new QTableWidgetItem("--"));

            //电量
            ui->tableAGVs->setItem(irow, (int)EAGVIndex::AGV_Electric, new QTableWidgetItem("--"));
            //置信度
            ui->tableAGVs->setItem(irow, (int)EAGVIndex::AGV_Confidence, new QTableWidgetItem("--"));
            //同步状态
            ui->tableAGVs->setItem(irow, (int)EAGVIndex::AGV_SyncStatus, new QTableWidgetItem("--"));

        }
        else
        {
            //IP
            ui->tableAGVs->setItem(irow, (int)EAGVIndex::AGV_IP, new QTableWidgetItem(itAGV.second.strAGVIP.c_str()));
            //电量
            ui->tableAGVs->setItem(irow, (int)EAGVIndex::AGV_Electric, new QTableWidgetItem(QString("%1").arg(itAGV.second.iElectricity)));
            //置信度
            ui->tableAGVs->setItem(irow, (int)EAGVIndex::AGV_Confidence, new QTableWidgetItem(QString("%1").arg(itAGV.second.iElectricity)));
            //同步状态
            strTemp = itAGV.second.bIsSync? QStringLiteral("已同步") :QStringLiteral("未同步");
            ui->tableAGVs->setItem(irow, (int)EAGVIndex::AGV_SyncStatus, new QTableWidgetItem(strTemp));

        }
        irow++;
    }
}


void CAGVListWindow::on_tableAGVs_cellClicked(int row, int column)
{
    //WIFI信息清空

    //控制器信息显示
    QTableWidgetItem *pItem = ui->tableAGVs->item(row, (int)EAGVIndex::AGV_ID);
    if (pItem != nullptr)
    {
        TAGVInfo tInfo;
        CAGVManager::GetInstance()->GetAGV(pItem->text().toStdString());
        ui->label_CPU->setText(QString("%1").arg(tInfo.iCPU));
        ui->label_MEM->setText(QString("%1").arg(tInfo.iRAM));
        ui->label_Ver->setText(tInfo.strVersion.c_str());
    }

}

void CAGVListWindow::on_tableAGVs_itemClicked(QTableWidgetItem *item)
{

}
