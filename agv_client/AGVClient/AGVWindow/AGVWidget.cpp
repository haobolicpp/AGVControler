#include <QMessageBox>
#include <QFileDialog>
#include <QDateTime>
#include "AGVWidget.h"
#include "ui_AGVWidget.h"
#include "mainwindow.h"
#include "AGVClient_def.h"
#include "NetManager.h"
#include "agv_msg_def.h"
#include "MapManager.h"
#include "MapView/MapItem.h"
#include "AGVManager.h"
#include "GoerToolUtile.h"
#include "CoreControl.h"
#include "WaitingWidgetControl.h"
#include "Commands/CommandManager.h"
#include "GlogWrapper.h"
#include "ManualControl.h"
#include "UpMapWidget.h"

CAGVWidget::CAGVWidget(MainWindow *pwnd, QWidget *parent) :
    CWidgetBase(parent),
    ui(new Ui::CAGVWidget)
{
    ui->setupUi(this);
    m_pManualCtrlWdget = nullptr;
    m_pMainWnd = pwnd;
    ui->tabWidget->tabBar();
    m_pWaitingWdgt = new CWaitingWidgetControl(this);

    ui->comb_direct->addItem("起点到终点");
    ui->comb_direct->addItem("终点到起点");
    ui->comb_direct->addItem("双向");

    //地图展示
    m_pMapScene = new CMapGraphicsScene();
    m_playoutMapView = new QGridLayout(this);
    m_pMapView = new CMapGraphicsView(m_pMapScene, this);
    m_playoutMapView->addWidget(m_pMapView);
    ui->widget_MapView->setLayout(m_playoutMapView);
    ui->stackedWidget->setHidden(true);

    //tabwidget信号槽绑定
    connect(ui->tabWidget, &QTabWidget::currentChanged, this, &CAGVWidget::SlotTabCurrentChanged);
    //滑块信号槽绑定
    connect(ui->EraserSlider , SIGNAL(valueChanged(int)), this, SLOT(EraserSlot(int)));
    //其他信号槽
    connect(m_pMapView, &CMapGraphicsView::SignalMapPress, this, &CAGVWidget::SlotUpdateMapInfo);
    connect(m_pMapView, &CMapGraphicsView::SignalLinePress, this, &CAGVWidget::SlotUpdatePathInfo);
    connect(m_pMapView, &CMapGraphicsView::SignalLineModify, this, &CAGVWidget::SlotUpdatePathInfo);
    connect(m_pMapView, &CMapGraphicsView::SignalStationPress, this, &CAGVWidget::SlotStationSelected);
    connect(m_pMapView, &CMapGraphicsView::SignalStationModify, this, &CAGVWidget::SlotUpdateStationInfo);

CNetManager::GetInstance()->ListenMsg(Class_Data, Type_Data_ReportGridMap, [&](TCallBackData* pdata){
    TResponseFunc tFunc = {
    [&](TCallBackData* pdata)
    {this->OnOneFrameMap(pdata);
     DeleteCallBackData(pdata);}
    ,
    pdata};
    QVariant qv;
    qv.setValue(tFunc);
    this->EmitNetResponseProc(qv);
});

    REGISTER_CALLBACK(Class_Data, Type_Data_ReportGridMap, this->OnOneFrameMap);
    REGISTER_CALLBACK(Class_Data,  Type_Data_StartScanMap, this->OnRecvStartScanResponse);
    REGISTER_CALLBACK(Class_Data,  Type_Data_StopScanMap, this->OnRecvEndScanResponse);
    REGISTER_CALLBACK(Class_Data,  Type_Data_DownMapBagData, this->OnRecvDownloadMapResponse);
    REGISTER_CALLBACK(Class_Control,  Type_Control_GoStation, this->OnRecvToDesResponse);
    REGISTER_CALLBACK(Class_Control,  Type_Control_StopGoStation, this->OnRecvCancelToDesResponse);
    REGISTER_CALLBACK(Class_Control,  Type_Control_GlobalPath, this->OnRecvGlobalPath);

    //地图拉取界面初始化
    m_pUpMapWidget = new CUpMapWidget();
    m_pUpMapWidget->hide();
}

CAGVWidget::~CAGVWidget()
{
    delete ui;
}

void CAGVWidget::InitData()
{
    CCoreControl::GetInstance()->SetCurrentMapModify(false);
    m_pMapView->LoadAGVMap();
    SlotUpdateMapInfo();
    ui->stackedWidget_Info->setCurrentIndex(0);
    ui->tabWidget->setCurrentIndex(0);
    ui->comb_direct->setEnabled(false);

    //禁用地图编辑按钮以外的按钮
    if (CAGVManager::GetInstance()->GetCurrentAGV()->eStatus == EConnectStatus::Connected_Invalid ||
            CAGVManager::GetInstance()->GetCurrentAGV()->eStatus == EConnectStatus::Connected_False)
    {
        DisableWidgetChildButtons(ui->tabWidget, QObjectList());
        EnableWidgetChildButtons(ui->tab_MapEdit);
        ui->toolButton_LoadMap->setEnabled(true);//启用加载地图按钮

        //测试2022-1-12
        ui->toolButton_UpMap->setEnabled(true); //启动地图下载按钮
    }
    else
    {
        EnableWidgetChildButtons(ui->tabWidget);
    }

    ui->toolButton_ManualControl->setEnabled(true);
}

void CAGVWidget::AGVDisconnect()
{
    //UI按钮重置
    EAGVRunStatus eRunStatus = CCoreControl::GetInstance()->GetAGVRunStatus();
    if (eRunStatus==EAGVRunStatus::Scan_Start ||
            eRunStatus==EAGVRunStatus::Scan_RecvMap)
    {
        ui->toolButton_scanmap->setText("构建地图");
    }
    if (eRunStatus == EAGVRunStatus::Run_GoToDes)
    {
        ui->btnGoToStation->setText("站点导航");
        m_pMapView->SetMouseMode(EMouseMode::MouseMode_Pointer);
    }

    //禁用地图编辑按钮以外的按钮
    DisableWidgetChildButtons(ui->tabWidget, QObjectList());
    EnableWidgetChildButtons(ui->tab_MapEdit);
    ui->btn_BackHome->setEnabled(true);
    //AGV运行状态重置
    CCoreControl::GetInstance()->SetAGVRunStatus(EAGVRunStatus::DoNothing);
    m_pMapView->AGVDisconnect();

    m_pMapView->SetMouseMode(EMouseMode::MouseMode_Pointer);

    m_pWaitingWdgt->Close();

    QMessageBox::information(this,"", "AGV掉线！");
}

void CAGVWidget::on_btn_BackHome_clicked()
{
    //检查是否需要保存地图
    //1.被编辑了
    //2.加载了新的地图
    if (CCoreControl::GetInstance()->GetCurrentMapModify())
    {
        QMessageBox::StandardButton btn =
                QMessageBox::information(this, "保存地图", "地图已修改，是否保存地图？",
                              QMessageBox::StandardButton::Ok, QMessageBox::StandardButton::Cancel);
        if (btn == QMessageBox::StandardButton::Ok)
        {
            if (0 == CMapManager::GetInstance()->SaveMap())
            {
                QMessageBox::information(this, "", "保存地图成功！");
            }
        }
    }

    //清空命令队列
    CCommandManager::GetInstance()->Clear();
    //切换主页面
    m_pMainWnd->ChangeToHome();
}

void CAGVWidget::on_toolButton_pointer_clicked()
{
    m_pMapView->SetMouseMode(EMouseMode::MouseMode_Pointer);
    ui->stackedWidget->setHidden(true);
}

void CAGVWidget::on_toolButton_station_clicked()
{
    ui->stackedWidget->show();
    ui->stackedWidget->setCurrentIndex(1);
}

void CAGVWidget::on_toolButton_line_clicked()
{
    m_pMapView->SetMouseMode(EMouseMode::MouseMode_Line);
    ui->stackedWidget->setHidden(true);
}

void CAGVWidget::on_toolButton_bezier_clicked()
{
    m_pMapView->SetMouseMode(EMouseMode::MouseMode_Bezier);
    ui->stackedWidget->setHidden(true);
}

void CAGVWidget::OnOneFrameMap(TCallBackData *pData)
{
    //更新地图内存
    TReportGripMap *pMap = (TReportGripMap*)pData->pData;
    CMapManager::GetInstance()->UpdateScanMap(pMap);

    //更新扫图状态
    CCoreControl::GetInstance()->UpdateStatusByRecvScanMap();

    //更新绘图数据
    m_pMapView->m_pmapItem->UpdatePainterMap(true);

    //刷新页面
    m_pMapView->update();
}

void CAGVWidget::OnRecvStartScanResponse(TCallBackData *pData)
{
    //判断是否可以继续扫图

    //可以扫图
    CCoreControl::GetInstance()->StartScanMap(true);
    m_pMapView->ResetMap();
}

void CAGVWidget::OnRecvEndScanResponse(TCallBackData *pData)
{
    TReportGripAndStreamMap *pMap = (TReportGripAndStreamMap*)pData->pData;

    m_bIsRecvAGVResponse = true;
    m_pWaitingWdgt->Close();

    EnableWidgetChildButtons(ui->tabWidget);
    ui->btn_BackHome->setEnabled(true);

    if (pData->tHeader.uiLen == 0)
    {
        QMessageBox::information(this, "", "地图为空！");
        CCoreControl::GetInstance()->StartScanMap(false);
        //更新绘图数据
        m_pMapView->m_pmapItem->UpdatePainterMap(false);
        //free(pMap);
        return;
    }

    //更新地图内存
    CMapManager::GetInstance()->UpdateScanStopMap(pMap);

    //弹窗提示是否保存地图
    ShowIsSaveMapWindow(true);

    CCoreControl::GetInstance()->StartScanMap(false);
    //更新绘图数据
    m_pMapView->m_pmapItem->UpdatePainterMap(false);
    ui->toolButton_scanmap->setText("构建地图");

    //free (pMap);
}

void CAGVWidget::OnRecvDownloadMapResponse(TCallBackData *pData)
{
    m_bIsRecvAGVResponse = true;
    m_pWaitingWdgt->Close();
}

void CAGVWidget::OnRecvToDesResponse(TCallBackData *pData)
{
    TAGVGoToDesResponse *pRes = (TAGVGoToDesResponse*)pData->pData;

    if (pRes == nullptr)
    {
        QMessageBox::information(this, "", "收到目标点到达消息，但数据为空！");
        return;
    }

    //切换状态并修改UI
    CCoreControl::GetInstance()->SetAGVRunStatus(EAGVRunStatus::DoNothing);
    EnableWidgetChildButtons(ui->tabWidget);
    ui->btn_BackHome->setEnabled(true);
    ui->btnGoToStation->setText("站点导航");
    m_pMapView->SetMouseMode(EMouseMode::MouseMode_Pointer);

    if (pRes->chFlag == 0)
    {
        QMessageBox::information(this, "", "收到目标点到达消息：成功到达目标站点");
    }
    else if(pRes->chFlag == 1)
    {
        QMessageBox::information(this, "", "收到目标点到达消息：路径无效或Move_base异常，未到达站点");
    }
    else
    {
        QMessageBox::information(this, "", "收到目标点到达消息：AGV繁忙");
    }


    //free (pRes);
}

void CAGVWidget::OnRecvCancelToDesResponse(TCallBackData *pData)
{
    GLOG_INFO << "RecvCancelToDesResponse";
    m_bIsRecvAGVResponse = true;
    m_pWaitingWdgt->Close();
    m_pMapView->SetMouseMode(EMouseMode::MouseMode_Pointer);
    CCoreControl::GetInstance()->SetAGVRunStatus(EAGVRunStatus::DoNothing);
}

void CAGVWidget::OnRecvGlobalPath(TCallBackData *pData)
{
    int iPtNum = *(int*)pData->pData;

    std::vector<TPointd> vecPath;
    for (int i=0; i<8*iPtNum; i+=8)
    {
        float fx = *(float*)((char*)pData->pData+4+i);
        float fy = *(float*)((char*)pData->pData+4+i+4);
        TPointd tPt;
        CMapManager::GetInstance()->WorldToMap(fx,fy,tPt.dX, tPt.dY);
        vecPath.push_back(tPt);
    }

    //增加图元
    m_pMapView->AddGlobalPathItem(vecPath);

    //free(pData->pData);
}

void CAGVWidget::DisableWidgetChildButtons(QWidget *pParentWidget, QObjectList listExclude)
{
    //非递归访问子控件
    QObjectList listTemp;
    listTemp.append(pParentWidget->children());
    while (!listTemp.empty())
    {
        QObject *pHeader = listTemp.at(0);

        bool bHaveExclued = false;
        for (auto obj : listExclude)
        {
            if (obj == pHeader)
            {
                bHaveExclued = true;
                break;
            }
        }
        if (bHaveExclued)
        {
            listTemp.pop_front();
            continue;
        }

        if (pHeader->isWidgetType())
        {
            QToolButton *pBtn = dynamic_cast<QToolButton*>(pHeader);
            if (pBtn != nullptr)
            {
                pBtn->setEnabled(false);
            }

            listTemp.append(pHeader->children());
        }

        listTemp.pop_front();
    }
}

void CAGVWidget::EnableWidgetChildButtons(QWidget *pParentWidget)
{
    //非递归访问子控件
    QObjectList listTemp;
    listTemp.append(pParentWidget->children());
    while (!listTemp.empty())
    {
        QObject *pHeader = listTemp.at(0);

        if (pHeader->isWidgetType())
        {
            QWidget *pWdg = (QWidget*)pHeader;
            pWdg->setEnabled(true);
            listTemp.append(pHeader->children());
        }

        listTemp.pop_front();
    }
}

void CAGVWidget::ShowIsSaveMapWindow(bool bScan)
{
    QMessageBox::StandardButton btn =
            QMessageBox::information(this, "保存地图", "是否保存地图？",
                          QMessageBox::StandardButton::Ok, QMessageBox::StandardButton::Cancel);
    if (btn == QMessageBox::StandardButton::Ok)
    {
        int iRet = 0;
        if (bScan)
        {
            CMapManager::GetInstance()->ScanMapCoverCurrentMap();//覆盖地图
            QString strAGVsDir = CGoerToolUtile::GetExeDirectory() + "/AGVs";
            QString strMapName = QDateTime::currentDateTime().toString("yyyyMMddhhmmss");
            strAGVsDir = strAGVsDir + "/" + CCoreControl::GetInstance()->GetCurrentMapAGVID().c_str() + "/maps/"+strMapName+".json";
            iRet = CMapManager::GetInstance()->SaveMap(strAGVsDir, strMapName);
        }
        else
        {
            iRet = CMapManager::GetInstance()->SaveMap();
        }
        if (0 == iRet)
        {
            QMessageBox::information(this, "", "保存地图成功！");
            if (bScan)
            {
                m_pMapView->ResetMap();
            }
            CCoreControl::GetInstance()->SetCurrentMapModify(false);
        }
    }
    else
    {
        //清空命令队列
        CCommandManager::GetInstance()->Clear();
        CCoreControl::GetInstance()->SetCurrentMapModify(false);
        CMapManager::GetInstance()->ReadMapJsonFromAGVDir(CAGVManager::GetInstance()->GetCurrentAGV()); //重新读取数据
        m_pMapView->LoadAGVMap();
        SlotUpdateMapInfo();
    }
}

void CAGVWidget::SlotUpdateMapInfo()
{
    ui->stackedWidget_Info->setCurrentIndex(0);
    TMapInfo & tmap = CMapManager::GetInstance()->GetCurrentMapInfo();
    ui->label_map_name->setText(tmap.strMapName.c_str());
    ui->label_map_width->setText(QString("%1").arg(tmap.iWidth));
    ui->label_map_height->setText(QString("%1").arg(tmap.iHight));
    ui->label_map_resolution->setText(QString().sprintf("%.2f", tmap.dResolution));
}

void CAGVWidget::SlotStationSelected(int iStationID)
{
    if (m_pMapView->GetMouseMode() == EMouseMode::MouseMode_ToDes)
    {
        if (CCoreControl::GetInstance()->GetAGVRunStatus() == EAGVRunStatus::Run_GoToDes)
        {
            QMessageBox::warning(this, "警告", "正在前往站点");
            return;
        }
        ui->btnGoToStation->setText("停止导航");
        CCoreControl::GetInstance()->SetAGVRunStatus(EAGVRunStatus::Run_GoToDes);
        //禁用tab栏所有控件
        DisableWidgetChildButtons(ui->tabWidget,QObjectList{ui->btnGoToStation});
        ui->btn_BackHome->setEnabled(false);
        //发送目标站点
        TAGVGoToDes tData;
        TStation *ptStation = CMapManager::GetInstance()->GetStation(iStationID);
        tData.iStationID = iStationID;
        double dx, dy;
        CMapManager::GetInstance()->MapToWorld(ptStation->tPt.dX, ptStation->tPt.dY, dx, dy);
        tData.fX = dx;
        tData.fY = dy;
        tData.fAngle = CGeometryAlgorithm::DegToRad(ptStation->dAngle);
        GLOG_INFO << "sendX:" << tData.fX << " sendY:" << tData.fY << " Angle:" << tData.fAngle;
        CNetManager::GetInstance()->SendGoToDes(CCoreControl::GetInstance()->GetCurrentMapAGVID(), tData);
    }
    else
    {
        SlotUpdateStationInfo(iStationID);
    }
}

void CAGVWidget::SlotUpdateStationInfo(int iStationID)
{
    TStation *pStation = CMapManager::GetInstance()->GetStation(iStationID);
    if (pStation != nullptr)
    {
        ui->stackedWidget_Info->setCurrentIndex(1);
        ui->label_st_id->setText(QString::number(pStation->iStationID));
        ui->label_st_x->setText(QString().sprintf("%.2f", pStation->tPt.dX));
        ui->label_st_y->setText(QString().sprintf("%.2f", pStation->tPt.dY));
        double dwx,dwy;
        CMapManager::GetInstance()->MapToWorld(pStation->tPt.dX, pStation->tPt.dY, dwx, dwy);
        ui->label_st_worldx->setText(QString().sprintf("%.4f", dwx));
        ui->label_st_worldy->setText(QString().sprintf("%.4f", dwy));
        ui->label_st_angle->setText(QString().sprintf("%.2f", pStation->dAngle));

        if (pStation->eType == EStationType::StationType_General)
        {
            ui->label_st_type->setText("普通");
        }
        else if (pStation->eType == EStationType::StationType_Charge)
        {
            ui->label_st_type->setText("充电");
        }
        else
        {
            ui->label_st_type->setText("未知");
        }
    }
}

void CAGVWidget::SlotUpdatePathInfo(int iPathID)
{
    TPath *pPath = CMapManager::GetInstance()->GetPath(iPathID);
    if (pPath != nullptr)
    {
        m_iSelectedPathID = iPathID;
        ui->stackedWidget_Info->setCurrentIndex(2);
        ui->label_path_ID->setText(QString::number(pPath->iPathID));
        ui->label_path_startid->setText(QString::number(pPath->iStartStation));
        ui->label_path_endid->setText(QString::number(pPath->iEndStation));
        ui->comb_direct->setCurrentIndex((int)pPath->eDirect);
        if (pPath->eType == EPathType::PathType_Line)
        {
            ui->label_path_type->setText("直线");
        }
        else if (pPath->eType == EPathType::PathType_Bezier)
        {
            ui->label_path_type->setText("贝塞尔曲线");
        }
        else if (pPath->eType == EPathType::PathType_Circle)
        {
            ui->label_path_type->setText("圆");
        }
        else
        {
            ui->label_path_type->setText("未知");
        }
    }
}

void CAGVWidget::SlotTabCurrentChanged(int index)
{
    static int ilastIndex = -1;
    if (ilastIndex == -1)
    {
        ilastIndex = index;
    }
    else
    {
        if (index == 3)
        {
            if (CCoreControl::GetInstance()->GetAGVRunStatus() != EAGVRunStatus::DoNothing)
            {
                QMessageBox::information(this, "", "当前不可编辑地图！");
                ui->tabWidget->setCurrentIndex(ilastIndex);
                return;
            }
            CCoreControl::GetInstance()->SetEditMap(true);
            m_pMapView->DeleGolbalPathItem();//清除全局路线item
            ui->comb_direct->setEnabled(true);
        }
        else
        {
            ui->comb_direct->setEnabled(false);
            CCoreControl::GetInstance()->SetEditMap(false);
            if (ilastIndex == 3)
            {
                //从地图编辑离开
                if (CCoreControl::GetInstance()->GetCurrentMapModify())
                {
                    ShowIsSaveMapWindow(false);
                }
                //暂时隐藏
                ui->stackedWidget->hide();
                m_pMapView->SetMouseMode(EMouseMode::MouseMode_Pointer);
            }
        }
        ilastIndex = index;
    }
}

void CAGVWidget::on_toolButton_scanmap_clicked()
{
    bool bScanMap = false;
    if (!CCoreControl::GetInstance()->IsScanMap())
    {
        bScanMap = true;
        ui->toolButton_scanmap->setText("停止建图");
        DisableWidgetChildButtons(ui->tabWidget,QObjectList{ui->toolButton_scanmap});
        ui->toolButton_ManualControl->setEnabled(true);
        ui->btn_BackHome->setEnabled(false);

        m_pMapView->SetMouseMode(EMouseMode::MouseMode_Pointer);
    }
    else
    {
        ui->toolButton_scanmap->setText("构建地图");
    }
    CNetManager::GetInstance()->SendScanMapReq(CCoreControl::GetInstance()->GetCurrentMapAGVID(), bScanMap);

    if (!bScanMap)
    {
        //弹窗等待
        m_bIsRecvAGVResponse = false;
        m_pWaitingWdgt->Show( "等待停止建图应答...");
    }

//    //测试
//    TCallBackData *pData = new TCallBackData;
//    OnEndScan(pData);
}

void CAGVWidget::on_toolButton_Del_clicked()
{
    m_pMapView->DeleteSelectedItems();
}

void CAGVWidget::on_toolButton_SaveMap_clicked()
{
    if ( 0 == CMapManager::GetInstance()->SaveMap())
    {
        CCoreControl::GetInstance()->SetCurrentMapModify(false);
        QMessageBox::information(this, "", "保存地图成功！");
    }
    else
    {
        QMessageBox::information(this, "", "保存地图失败！");
    }
}

//下载地图
void CAGVWidget::on_toolButton_DownMap_clicked()
{
    std::string strAGVID = CCoreControl::GetInstance()->GetCurrentMapAGVID();
    TAGVInfo *pAGV = CAGVManager::GetInstance()->GetAGV(strAGVID);

    //发送指令
    CNetManager::GetInstance()->SendMapData(strAGVID, pAGV->strMapJson.c_str(), pAGV->strMapJson.size());

    //弹窗等待
    m_bIsRecvAGVResponse = false;
    m_pWaitingWdgt->Show( "下发地图包中...");
}

//上拉地图
void CAGVWidget::on_toolButton_UpMap_clicked()
{
    //弹出对话框选择需要上拉的地图
    if(NULL == m_pUpMapWidget)
    {
        m_pUpMapWidget = new CUpMapWidget();
        m_pUpMapWidget->show();
    }
    else
    {
        m_pUpMapWidget->show();
    }
}

//从本地加载一张指定的地图
void CAGVWidget::on_toolButton_LoadMap_clicked()
{
    QString strAGVsDir = CGoerToolUtile::GetExeDirectory() + "/AGVs";
    strAGVsDir = strAGVsDir + "/" + CCoreControl::GetInstance()->GetCurrentMapAGVID().c_str() + "/maps/";
    QString fileName;

    QFileDialog fileDialog(this);
    fileDialog.setWindowTitle(QStringLiteral("选中文件"));
    fileDialog.setDirectory(strAGVsDir);
    fileDialog.setNameFilter(tr("File(*.json)"));//设置文件过滤器
    fileDialog.setFileMode(QFileDialog::ExistingFile); //模式：QFileDialog::ExistingFiles多个文件 ExistingFile单个文件
    fileDialog.setViewMode(QFileDialog::Detail);//设置视图模式

    if (!fileDialog.exec()) {
        return;
    }
    QStringList filelist = fileDialog.selectedFiles();
    if (0 == filelist.size())
    {
        QMessageBox::information(this, "", "未选中文件");
        return;
    }
    fileName = filelist.at(0);
    //读取地图数据
    int iret = CMapManager::GetInstance()->ReadMapJsonFromSpecifyFile(fileName);
    if (iret == (int)EAGVManagerErr::JsonError)
    {
        QMessageBox::information(this, "", "地图格式错误！");
        return;
    }
    else if (iret == (int)EAGVManagerErr::FileNotExist)
    {
        QMessageBox::information(this, "", "文件不存在！");
        return;
    }else{}

    //重新加载地图
    m_pMapView->LoadAGVMap();

    CCoreControl::GetInstance()->SetCurrentMapModify(true);
}

void CAGVWidget::on_toolButton_undo_clicked()
{
    //清除选中状态，否则可能会有问题
    m_pMapView->ClearItemsSelectedStatus();
    if (!CCommandManager::GetInstance()->undo())
    {
        //后期改为失能 TODO
        QMessageBox::information(this, "", "无法再撤销了！");
    }
}

void CAGVWidget::on_toolButton_redo_clicked()
{
    //清除选中状态，否则可能会有问题
    m_pMapView->ClearItemsSelectedStatus();
    if (!CCommandManager::GetInstance()->redo())
    {
        //后期改为失能 TODO
        QMessageBox::information(this, "", "无法再回退了！");
    }
}

void CAGVWidget::on_toolButton_eraser_clicked()
{
    m_pMapView->SetMouseMode(EMouseMode::MouseMode_Eraser);
    ui->stackedWidget->show();
    ui->stackedWidget->setCurrentIndex(0);
}

void CAGVWidget::EraserSlot(int iValue)
{
    m_pMapView->SetEraserSize(iValue);
}

void CAGVWidget::on_pushButton_add_station_clicked()
{
    m_pMapView->SetMouseMode(EMouseMode::MouseMode_Station);
}

void CAGVWidget::on_pushButton_add_station_agv_clicked()
{
    if (!m_pMapView->AddStationOnAGVPos())
    {
        QMessageBox::warning(this, "", "添加站点失败，AGV不在线");
    }
}

void CAGVWidget::on_pushButton_mov_stationtoagv_clicked()
{
    TAGVInfo *pAGV = CAGVManager::GetInstance()->GetCurrentAGV();
    if (pAGV->eStatus != EConnectStatus::Connected_Sucess)
    {
        QMessageBox::warning(this, "", "移动站点失败，AGV不在线");
    }

    if (!m_pMapView->MoveStationToAGV(pAGV))
    {
        QMessageBox::warning(this, "", "未选中站点！");
    }
}

void CAGVWidget::on_pushButton_clicked()
{
    m_pMapView->ClearOutlier();
}

void CAGVWidget::on_btnGoToStation_clicked()
{
    if (ui->btnGoToStation->text() == "停止导航")
    {
        ui->btnGoToStation->setText("站点导航");
        m_pMapView->SetMouseMode(EMouseMode::MouseMode_Pointer);

        EnableWidgetChildButtons(ui->tabWidget);
        ui->btn_BackHome->setEnabled(true);

        //没有下发站点，则取消导航
        if (CCoreControl::GetInstance()->GetAGVRunStatus() != EAGVRunStatus::Run_GoToDes)
        {
            return;
        }

        //下发停止导航指令
        CNetManager::GetInstance()->SendCancelGoToDes(CCoreControl::GetInstance()->GetCurrentMapAGVID());
        //弹窗等待
        m_bIsRecvAGVResponse = false;
        m_pWaitingWdgt->Show( "等待停止导航应答...");
    }
    else
    {
        m_pMapView->SetMouseMode(EMouseMode::MouseMode_ToDes);
    }
}

void CAGVWidget::on_comb_direct_currentIndexChanged(int index)
{
    TPath *pPath = CMapManager::GetInstance()->GetPath(m_iSelectedPathID);
    if (pPath != nullptr)
    {
        pPath->eDirect = (EPathDirect)ui->comb_direct->currentIndex();
        //地图修改标记
        CCoreControl::GetInstance()->SetCurrentMapModify(true);
        m_pMapView->update();
    }
}

//手动控制
void CAGVWidget::on_toolButton_ManualControl_clicked()
{
    if (m_pManualCtrlWdget == nullptr)
    {
        m_pManualCtrlWdget = new CManualControlWidget(this);
    }

    if (m_pManualCtrlWdget->isHidden())
    {
        m_pManualCtrlWdget->ShowCtrl();
    }
}
