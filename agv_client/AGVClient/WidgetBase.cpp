#include <functional>
#include "WidgetBase.h"
#include "NetManager.h"

CWidgetBase::CWidgetBase(QWidget *parent) : QWidget(parent)
{
    connect(this, &CWidgetBase::SignalNetResponseProc, this, &CWidgetBase::SlotNetResponseProc,
            Qt::ConnectionType::QueuedConnection);
}

void CWidgetBase::EmitNetResponseProc(QVariant qv)
{
    emit SignalNetResponseProc(qv);
}

void CWidgetBase::SlotNetResponseProc(QVariant qv)
{
    TResponseFunc tFunc = qv.value<TResponseFunc>();
    tFunc.callBack(tFunc.pData);
}
