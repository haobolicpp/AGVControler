#ifndef MANUALCONTROL_H
#define MANUALCONTROL_H

#include <QWidget>
#include <QPushButton>
#include <QKeyEvent>
#include <QTimer>
#include "WidgetBase.h"

namespace Ui {
class CManualControl;
}

class CWaitingWidgetControl;
class CManualControlWidget : public CWidgetBase
{
    Q_OBJECT

public:
    explicit CManualControlWidget(QWidget *parent = nullptr);
    ~CManualControlWidget();

    void ShowCtrl();
    void HideCtrl();

private:
    void BtnPressedPri(QPushButton *pBtn, bool bPressed);
    void keyPressEvent(QKeyEvent *event) override;
    void keyReleaseEvent(QKeyEvent *event) override;
    void closeEvent(QCloseEvent *event) override;

    void OnTeleoperation(TCallBackData *pData);

private:
    Ui::CManualControl *ui;

    /*按键是否按下的数据
    第0位：上
    第1位：下
    第2位：左
    第3位：右
    0表示未按下，1表示按下。
    发送频率50ms。
    */
    int m_iBtnData;

    QTimer *m_ptimerSend;

    CWaitingWidgetControl *m_pWaitingWdgt;

    //是否继续处理应答逻辑
    bool m_bPorcCallback;
};

#endif // MANUALCONTROL_H
