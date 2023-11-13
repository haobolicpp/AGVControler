#include "PropertyWidget.h"
#include "ui_PropertyWidget.h"

CPropertyWidget::CPropertyWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::CPropertyWidget)
{
    ui->setupUi(this);
}

CPropertyWidget::~CPropertyWidget()
{
    delete ui;
}
