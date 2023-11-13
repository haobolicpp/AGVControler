#ifndef PROPERTYWIDGET_H
#define PROPERTYWIDGET_H

#include <QWidget>

namespace Ui {
class CPropertyWidget;
}

class CPropertyWidget : public QWidget
{
    Q_OBJECT

public:
    explicit CPropertyWidget(QWidget *parent = nullptr);
    ~CPropertyWidget();

private:
    Ui::CPropertyWidget *ui;
};

#endif // PROPERTYWIDGET_H
