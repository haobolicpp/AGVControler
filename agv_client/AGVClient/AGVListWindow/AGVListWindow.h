#ifndef AGVLISTWINDOW_H
#define AGVLISTWINDOW_H
#include <QTimer>
#include <QWidget>
#include <QTableWidgetItem>
#include "AGVWindow/AGVWidget.h"
#include "WidgetBase.h"

namespace Ui {
class CAGVListWindow;
}

class MainWindow;
class CAGVListWindow : public CWidgetBase
{
    Q_OBJECT

public:
    explicit CAGVListWindow(MainWindow *pwnd, QWidget *parent = nullptr);
    ~CAGVListWindow();

signals:
    void SignalAGVConnect(QString strAGVID, QString strAGVIP, QVariant bSucess, void* pData);
    void SignalAGVDisconnect(QString strAGVID);

private slots:
    //收到AGV连接信息
    void SlotRecvAGVConnectInfo(QString strAGVID, QString strAGVIP, QVariant bSucess, void *pData);
    //收到AGV断线信息
    void SlotRecvAGVDisConnectInfo(QString strAGVID);
    void on_tableAGVs_itemDoubleClicked(QTableWidgetItem *item);
    void on_pushButton_ScanAGV_clicked();
    void on_pushButton_Delete_clicked();

    void on_tableAGVs_cellClicked(int row, int column);

    void on_tableAGVs_itemClicked(QTableWidgetItem *item);

private:
    //AGV列表更新
    void UpdateAGVList();
    //处理实时上报的AGV数据
    void RecvAGVReportData(TCallBackData *pData);

private:
    Ui::CAGVListWindow *ui;
    MainWindow *m_pMainWnd;
    QTimer m_timerUpdateAGVList;
};

#endif // AGVLISTWINDOW_H
