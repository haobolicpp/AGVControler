#ifndef AGVWIDGET_H
#define AGVWIDGET_H

#include <QWidget>
#include <QGridLayout>
#include "MapView/MapGraphicsView.h"
#include "MapView/MapGraphicsScene.h"
#include "WidgetBase.h"

namespace Ui {
class CAGVWidget;
}

class MainWindow;
class CWaitingWidgetControl;
class CManualControlWidget;
class CUpMapWidget;
class CAGVWidget : public CWidgetBase
{
    Q_OBJECT

public:
    explicit CAGVWidget(MainWindow *pwnd, QWidget *parent = nullptr);
    ~CAGVWidget();

    //初始化页面数据
    void InitData();

    //处理AGV断线
    void AGVDisconnect();

private slots:
    void on_btn_BackHome_clicked();
    void on_toolButton_pointer_clicked();
    void on_toolButton_station_clicked();
    void on_toolButton_line_clicked();
    void on_toolButton_bezier_clicked();
    void on_toolButton_scanmap_clicked();
    void on_toolButton_Del_clicked();
    void on_toolButton_SaveMap_clicked();
    void on_toolButton_DownMap_clicked();
    void on_toolButton_UpMap_clicked();
    void on_toolButton_LoadMap_clicked();
    void on_toolButton_undo_clicked();
    void on_toolButton_redo_clicked();
    void on_toolButton_eraser_clicked();
    void on_pushButton_add_station_clicked();
    void on_pushButton_add_station_agv_clicked();
    void on_pushButton_mov_stationtoagv_clicked();
    void on_btnGoToStation_clicked();
    void on_pushButton_clicked();

    void EraserSlot(int iValue);
    void SlotUpdateMapInfo();
    void SlotStationSelected(int iStationID);
    void SlotUpdateStationInfo(int iStationID);
    void SlotUpdatePathInfo(int iPathID);

    void SlotTabCurrentChanged(int index);

    void on_comb_direct_currentIndexChanged(int index);

    void on_toolButton_ManualControl_clicked();

private:
    //收到一帧数据
    void OnOneFrameMap(TCallBackData *pData);
    //收到开始扫图应答
    void OnRecvStartScanResponse(TCallBackData *pData);
    //收到停止扫图应答
    void OnRecvEndScanResponse(TCallBackData *pData);
    //收到下载地图包完成应答
    void OnRecvDownloadMapResponse(TCallBackData *pData);
    //收到目标点到达的应答
    void OnRecvToDesResponse(TCallBackData *pData);
    //收到停止导航的应答
    void OnRecvCancelToDesResponse(TCallBackData *pData);
    //收到全局路径
    void OnRecvGlobalPath(TCallBackData *pData);

    //禁用Widget child buttons
    //QObjectList listExclude: 排除的buttons
    void DisableWidgetChildButtons(QWidget *pParentWidget, QObjectList listExclude);
    void EnableWidgetChildButtons(QWidget *pParentWidget);

    //弹窗提示是否保存地图
    void ShowIsSaveMapWindow(bool bScan);


private:
    Ui::CAGVWidget *ui;
    MainWindow *m_pMainWnd;
    CMapGraphicsView *m_pMapView;
    CMapGraphicsScene *m_pMapScene;
    QGridLayout *m_playoutMapView;
    CUpMapWidget *m_pUpMapWidget;
    //弹窗等待
    CWaitingWidgetControl *m_pWaitingWdgt;
    //某些指令下发后，是否收到AGV控制器应答消息，注意多个指令公用该标记，因为这些指令不会同时执行
    bool m_bIsRecvAGVResponse;

    //当前选中的路线
    int m_iSelectedPathID;

    //控制窗口
    CManualControlWidget *m_pManualCtrlWdget;

    friend class MainWindow;
    friend class CAGVListWindow;
};

#endif // AGVWIDGET_H
