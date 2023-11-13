#ifndef CWIDGETBASE_H
#define CWIDGETBASE_H
#include <functional>
#include <QWidget>
#include <QVariant>
#include "NetManager.h"

//信号槽传递的数据
typedef struct TResponseFunc{
    ResponseCallFunc callBack; //槽执行的函数对象
    TCallBackData *pData; //函数对象传递的参数
} TResponseFunc;
Q_DECLARE_METATYPE(TResponseFunc);

//注册监听回调的宏,TCallBackData*已经在此处删除了
#define REGISTER_CALLBACK(class ,type, funcCallback) \
CNetManager::GetInstance()->ListenMsg(class, type, [&](TCallBackData* pdata){ \
    TResponseFunc tFunc = {[&](TCallBackData* pdata){funcCallback(pdata);DeleteCallBackData(pdata);}, pdata}; \
    QVariant qv; \
    qv.setValue(tFunc); \
    this->EmitNetResponseProc(qv); \
});

class CWidgetBase : public QWidget
{
    Q_OBJECT
public:
    explicit CWidgetBase(QWidget *parent = nullptr);

    //激发信号，处理收到的网络数据，将网络线程发的数据转移到UI线程中处理
    //qv中传递TResponseFunc类型的数据
    void EmitNetResponseProc(QVariant qv);

signals:
    void SignalNetResponseProc(QVariant qv);

public slots:
    void SlotNetResponseProc(QVariant qv);
};

#endif // CWIDGETBASE_H
