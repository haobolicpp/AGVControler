#ifndef UPMAPWIDGET_H
#define UPMAPWIDGET_H

#include <QWidget>

namespace Ui {
class CUpMapWidget;
}

class CUpMapWidget : public QWidget
{
    Q_OBJECT

public:
    explicit CUpMapWidget(QWidget *parent = nullptr);
    ~CUpMapWidget();

private:
    Ui::CUpMapWidget *ui;
};

#endif // UPMAPWIDGET_H
