#include "UpMapWidget.h"
#include "ui_UpMapWidget.h"
#include "HeaderViewEx.h"

CUpMapWidget::CUpMapWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::CUpMapWidget)
{
    ui->setupUi(this);

    QStringList strList;
    //地图列表初始化
    QHeaderView *ph = new QHeaderView(Qt::Horizontal, this);
    strList.clear();
    strList <<  QStringLiteral("地图名称") << QStringLiteral("是否使用");
    ui->UpMapTableWidget->setHorizontalHeader(ph);
    ui->UpMapTableWidget->verticalHeader()->setVisible(false);
    ui->UpMapTableWidget->setEditTriggers(QAbstractItemView::NoEditTriggers);  //禁用编辑
    ui->UpMapTableWidget->setSelectionBehavior(QAbstractItemView::SelectRows);  //整行选中
    ui->UpMapTableWidget->setColumnCount(strList.length());
    ui->UpMapTableWidget->setHorizontalHeaderLabels(strList);
    ui->UpMapTableWidget->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch); //表格自适应宽度

    //插入测试数据
    ui->UpMapTableWidget->setRowCount(10);
    ui->UpMapTableWidget->setItem(0, 0, new QTableWidgetItem("123456"));
    ui->UpMapTableWidget->setItem(0, 1, new QTableWidgetItem("是"));
    ui->UpMapTableWidget->setItem(1, 0, new QTableWidgetItem("abc"));
    ui->UpMapTableWidget->setItem(1, 1, new QTableWidgetItem("否"));
}

CUpMapWidget::~CUpMapWidget()
{
    delete ui;
}
