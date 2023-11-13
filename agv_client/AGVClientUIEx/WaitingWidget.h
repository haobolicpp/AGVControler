#ifndef CTRANSPARENTWIDGET_H
#define CTRANSPARENTWIDGET_H

#include <QWidget>
#include <QStackedLayout>
#include <QPainter>
#include <QEvent>

class CTransparentWidget : public QWidget
{
    Q_OBJECT
public:
    CTransparentWidget(QWidget *parent):QWidget(parent){};
    virtual ~CTransparentWidget(){};

    void paintEvent(QPaintEvent *event){
        QPainter painter(this);
        painter.setOpacity(0.5);
        painter.setBrush(QColor(0,0,0, 100));
        painter.drawRect(contentsRect());
        QWidget::paintEvent(event);
    };
};

class CWaitingWidget : public QWidget
{
    Q_OBJECT
public:
    explicit CWaitingWidget(QWidget *parent = nullptr);

    void setWindowLayout(QLayout *layout);

protected:
    void paintEvent(QPaintEvent *event) Q_DECL_OVERRIDE;
    bool event(QEvent *event) Q_DECL_OVERRIDE;
    bool eventFilter(QObject *obj, QEvent *event) Q_DECL_OVERRIDE;

private:
    //整体布局
    QVBoxLayout *m_pWinlayout;
    //中心widget
    CTransparentWidget *m_pCenterWdgt;
    //widget布局
    QLayout *m_layoutWdgt;

};

#endif // CTRANSPARENTWIDGET_H
