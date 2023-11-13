
#include <QCoreApplication>
#include "WaitingWidget.h"

CWaitingWidget::CWaitingWidget(QWidget *parent) : QWidget(parent)
{
    if (parent) {
        parent->installEventFilter(this);
    }

    setWindowOpacity(0.7);

    m_pWinlayout = new QVBoxLayout;
    setLayout(m_pWinlayout);

    m_layoutWdgt = new QVBoxLayout;
    m_pCenterWdgt = new CTransparentWidget(this);
    m_pCenterWdgt->setWindowOpacity(0.5);
    m_pWinlayout->addWidget(m_pCenterWdgt);
    m_pWinlayout->setAlignment(m_pCenterWdgt, Qt::AlignCenter);
    m_pWinlayout->setMargin(0);
    m_pWinlayout->setSpacing(0);
}

void CWaitingWidget::setWindowLayout(QLayout *layout)
{
    if (m_layoutWdgt != nullptr){
        delete m_layoutWdgt;
    }
    m_layoutWdgt = layout;
    m_layoutWdgt->setMargin(0);
    m_layoutWdgt->setSpacing(0);
    m_pCenterWdgt->setLayout(layout);
}

void CWaitingWidget::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);

    QBrush brush;
    brush.setStyle(Qt::SolidPattern);
    brush.setColor(Qt::black);
    painter.setBrush(brush);
    painter.setPen(Qt::NoPen);
    painter.setOpacity(0.5);//设置半透明
    painter.drawRect(rect());
}

bool CWaitingWidget::event(QEvent *event)
{
    if (!parent()) {
        return QWidget::event(event);
    }
    switch (event->type())
    {
    case QEvent::ParentChange:
    {
        parent()->installEventFilter(this);
        QWidget *widget = parentWidget();
        QRect rect;
        if (widget) {
            rect = widget->rect();
        }
        setGeometry(rect);
        break;
    }
    case QEvent::ParentAboutToChange:
    {
        parent()->removeEventFilter(this);
        break;
    }
    case QEvent::Move:
    case QEvent::Resize:
    {
        if (this->parentWidget()){
            //qDebug() << parentWidget()->geometry().x() << parentWidget()->geometry().y();
            // bob.li 强行设置到0,0位置，setGeometry是相对于父窗口的位置（父窗口有的话）
            this->setGeometry(0,0, parentWidget()->geometry().width(), parentWidget()->geometry().height());//跟随父窗口调整
        }
    }
    default:
        break;
    }
    return QWidget::event(event);
}

bool CWaitingWidget::eventFilter(QObject *obj, QEvent *event)
{
    switch (event->type())
    {
    case QEvent::Move:
    case QEvent::Resize:
    {
        QWidget *widget = parentWidget();
        QRect rect;
        if (widget) {
            rect = widget->rect();
        }
        setGeometry(rect);
        break;
    }
    default:
        break;
    }
    return QWidget::eventFilter(obj, event);
}
