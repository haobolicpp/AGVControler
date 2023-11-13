/********************************************************************************
** Form generated from reading UI file 'AGVWidget.ui'
**
** Created by: Qt User Interface Compiler version 5.12.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_AGVWIDGET_H
#define UI_AGVWIDGET_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QStackedWidget>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QToolButton>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_CAGVWidget
{
public:
    QGridLayout *gridLayout_2;
    QHBoxLayout *horizontalLayout;
    QWidget *widget_MapView;
    QSpacerItem *horizontalSpacer;
    QStackedWidget *stackedWidget_Info;
    QWidget *stackedWidget_InfoPage1;
    QGridLayout *gridLayout_3;
    QLabel *label_2;
    QLabel *label_map_width;
    QLabel *label_map_name;
    QLabel *label_map_height;
    QLabel *label_4;
    QLabel *label_1;
    QLabel *label_map_resolution;
    QLabel *label_3;
    QSpacerItem *verticalSpacer;
    QWidget *page_3;
    QGridLayout *gridLayout_5;
    QLabel *label_st_type;
    QLabel *label_st_worldx;
    QLabel *label_st_angle;
    QLabel *label_7;
    QLabel *label_9;
    QLabel *label_8;
    QLabel *label_5;
    QLabel *label_11;
    QLabel *label_st_id;
    QLabel *label_14;
    QLabel *label_st_x;
    QLabel *label_st_y;
    QSpacerItem *verticalSpacer_2;
    QLabel *label_10;
    QLabel *label_st_worldy;
    QWidget *page_4;
    QGridLayout *gridLayout_4;
    QLabel *label_13;
    QLabel *label_path_startid;
    QLabel *label_17;
    QLabel *label_15;
    QLabel *label_path_type;
    QLabel *label_19;
    QLabel *label_path_endid;
    QLabel *label_path_ID;
    QSpacerItem *verticalSpacer_3;
    QLabel *label_20;
    QComboBox *comb_direct;
    QWidget *widget;
    QGridLayout *gridLayout;
    QTabWidget *tabWidget;
    QWidget *tab_MapManager;
    QHBoxLayout *horizontalLayout_2;
    QToolButton *toolButton_scanmap;
    QToolButton *toolButton_LoadMap;
    QToolButton *toolButton_UpMap;
    QToolButton *toolButton_DownMap;
    QWidget *tab_AGVControl;
    QHBoxLayout *horizontalLayout_3;
    QToolButton *toolButton_5;
    QToolButton *btnGoToStation;
    QToolButton *toolButton_ManualControl;
    QWidget *tab_AGVParam;
    QHBoxLayout *horizontalLayout_4;
    QToolButton *toolButton_14;
    QWidget *tab_MapEdit;
    QHBoxLayout *horizontalLayout_8;
    QToolButton *toolButton_pointer;
    QToolButton *toolButton_station;
    QToolButton *toolButton_line;
    QToolButton *toolButton_bezier;
    QToolButton *toolButton_eraser;
    QFrame *line;
    QToolButton *toolButton_Del;
    QToolButton *toolButton_SaveMap;
    QToolButton *toolButton_map_check;
    QFrame *line_3;
    QToolButton *toolButton_undo;
    QToolButton *toolButton_redo;
    QToolButton *btn_BackHome;
    QStackedWidget *stackedWidget;
    QWidget *page;
    QHBoxLayout *horizontalLayout_6;
    QHBoxLayout *horizontalLayout_5;
    QLabel *label;
    QSlider *EraserSlider;
    QPushButton *pushButton;
    QSpacerItem *horizontalSpacer_2;
    QWidget *page_2;
    QHBoxLayout *horizontalLayout_9;
    QHBoxLayout *horizontalLayout_7;
    QPushButton *pushButton_add_station;
    QPushButton *pushButton_add_station_agv;
    QPushButton *pushButton_mov_stationtoagv;
    QSpacerItem *horizontalSpacer_3;

    void setupUi(QWidget *CAGVWidget)
    {
        if (CAGVWidget->objectName().isEmpty())
            CAGVWidget->setObjectName(QString::fromUtf8("CAGVWidget"));
        CAGVWidget->resize(818, 397);
        CAGVWidget->setMaximumSize(QSize(16777215, 16777215));
        gridLayout_2 = new QGridLayout(CAGVWidget);
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        widget_MapView = new QWidget(CAGVWidget);
        widget_MapView->setObjectName(QString::fromUtf8("widget_MapView"));

        horizontalLayout->addWidget(widget_MapView);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Fixed, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer);

        stackedWidget_Info = new QStackedWidget(CAGVWidget);
        stackedWidget_Info->setObjectName(QString::fromUtf8("stackedWidget_Info"));
        stackedWidget_Info->setMinimumSize(QSize(300, 0));
        stackedWidget_Info->setMaximumSize(QSize(200, 16777215));
        stackedWidget_InfoPage1 = new QWidget();
        stackedWidget_InfoPage1->setObjectName(QString::fromUtf8("stackedWidget_InfoPage1"));
        gridLayout_3 = new QGridLayout(stackedWidget_InfoPage1);
        gridLayout_3->setObjectName(QString::fromUtf8("gridLayout_3"));
        gridLayout_3->setVerticalSpacing(20);
        label_2 = new QLabel(stackedWidget_InfoPage1);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        gridLayout_3->addWidget(label_2, 1, 0, 1, 1);

        label_map_width = new QLabel(stackedWidget_InfoPage1);
        label_map_width->setObjectName(QString::fromUtf8("label_map_width"));

        gridLayout_3->addWidget(label_map_width, 1, 1, 1, 1);

        label_map_name = new QLabel(stackedWidget_InfoPage1);
        label_map_name->setObjectName(QString::fromUtf8("label_map_name"));

        gridLayout_3->addWidget(label_map_name, 0, 1, 1, 1);

        label_map_height = new QLabel(stackedWidget_InfoPage1);
        label_map_height->setObjectName(QString::fromUtf8("label_map_height"));

        gridLayout_3->addWidget(label_map_height, 2, 1, 1, 1);

        label_4 = new QLabel(stackedWidget_InfoPage1);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        gridLayout_3->addWidget(label_4, 0, 0, 1, 1);

        label_1 = new QLabel(stackedWidget_InfoPage1);
        label_1->setObjectName(QString::fromUtf8("label_1"));

        gridLayout_3->addWidget(label_1, 3, 0, 1, 1);

        label_map_resolution = new QLabel(stackedWidget_InfoPage1);
        label_map_resolution->setObjectName(QString::fromUtf8("label_map_resolution"));

        gridLayout_3->addWidget(label_map_resolution, 3, 1, 1, 1);

        label_3 = new QLabel(stackedWidget_InfoPage1);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        gridLayout_3->addWidget(label_3, 2, 0, 1, 1);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        gridLayout_3->addItem(verticalSpacer, 4, 0, 1, 1);

        stackedWidget_Info->addWidget(stackedWidget_InfoPage1);
        page_3 = new QWidget();
        page_3->setObjectName(QString::fromUtf8("page_3"));
        gridLayout_5 = new QGridLayout(page_3);
        gridLayout_5->setObjectName(QString::fromUtf8("gridLayout_5"));
        gridLayout_5->setVerticalSpacing(20);
        label_st_type = new QLabel(page_3);
        label_st_type->setObjectName(QString::fromUtf8("label_st_type"));

        gridLayout_5->addWidget(label_st_type, 1, 1, 1, 1);

        label_st_worldx = new QLabel(page_3);
        label_st_worldx->setObjectName(QString::fromUtf8("label_st_worldx"));

        gridLayout_5->addWidget(label_st_worldx, 4, 1, 1, 1);

        label_st_angle = new QLabel(page_3);
        label_st_angle->setObjectName(QString::fromUtf8("label_st_angle"));

        gridLayout_5->addWidget(label_st_angle, 6, 1, 1, 1);

        label_7 = new QLabel(page_3);
        label_7->setObjectName(QString::fromUtf8("label_7"));

        gridLayout_5->addWidget(label_7, 2, 0, 1, 1);

        label_9 = new QLabel(page_3);
        label_9->setObjectName(QString::fromUtf8("label_9"));

        gridLayout_5->addWidget(label_9, 3, 0, 1, 1);

        label_8 = new QLabel(page_3);
        label_8->setObjectName(QString::fromUtf8("label_8"));

        gridLayout_5->addWidget(label_8, 4, 0, 1, 1);

        label_5 = new QLabel(page_3);
        label_5->setObjectName(QString::fromUtf8("label_5"));

        gridLayout_5->addWidget(label_5, 0, 0, 1, 1);

        label_11 = new QLabel(page_3);
        label_11->setObjectName(QString::fromUtf8("label_11"));

        gridLayout_5->addWidget(label_11, 6, 0, 1, 1);

        label_st_id = new QLabel(page_3);
        label_st_id->setObjectName(QString::fromUtf8("label_st_id"));

        gridLayout_5->addWidget(label_st_id, 0, 1, 1, 1);

        label_14 = new QLabel(page_3);
        label_14->setObjectName(QString::fromUtf8("label_14"));

        gridLayout_5->addWidget(label_14, 1, 0, 1, 1);

        label_st_x = new QLabel(page_3);
        label_st_x->setObjectName(QString::fromUtf8("label_st_x"));

        gridLayout_5->addWidget(label_st_x, 2, 1, 1, 1);

        label_st_y = new QLabel(page_3);
        label_st_y->setObjectName(QString::fromUtf8("label_st_y"));

        gridLayout_5->addWidget(label_st_y, 3, 1, 1, 1);

        verticalSpacer_2 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        gridLayout_5->addItem(verticalSpacer_2, 7, 0, 1, 1);

        label_10 = new QLabel(page_3);
        label_10->setObjectName(QString::fromUtf8("label_10"));

        gridLayout_5->addWidget(label_10, 5, 0, 1, 1);

        label_st_worldy = new QLabel(page_3);
        label_st_worldy->setObjectName(QString::fromUtf8("label_st_worldy"));

        gridLayout_5->addWidget(label_st_worldy, 5, 1, 1, 1);

        stackedWidget_Info->addWidget(page_3);
        page_4 = new QWidget();
        page_4->setObjectName(QString::fromUtf8("page_4"));
        gridLayout_4 = new QGridLayout(page_4);
        gridLayout_4->setObjectName(QString::fromUtf8("gridLayout_4"));
        gridLayout_4->setVerticalSpacing(20);
        label_13 = new QLabel(page_4);
        label_13->setObjectName(QString::fromUtf8("label_13"));

        gridLayout_4->addWidget(label_13, 0, 0, 1, 1);

        label_path_startid = new QLabel(page_4);
        label_path_startid->setObjectName(QString::fromUtf8("label_path_startid"));

        gridLayout_4->addWidget(label_path_startid, 3, 1, 1, 1);

        label_17 = new QLabel(page_4);
        label_17->setObjectName(QString::fromUtf8("label_17"));

        gridLayout_4->addWidget(label_17, 4, 0, 1, 1);

        label_15 = new QLabel(page_4);
        label_15->setObjectName(QString::fromUtf8("label_15"));

        gridLayout_4->addWidget(label_15, 3, 0, 1, 1);

        label_path_type = new QLabel(page_4);
        label_path_type->setObjectName(QString::fromUtf8("label_path_type"));

        gridLayout_4->addWidget(label_path_type, 1, 1, 1, 1);

        label_19 = new QLabel(page_4);
        label_19->setObjectName(QString::fromUtf8("label_19"));

        gridLayout_4->addWidget(label_19, 1, 0, 1, 1);

        label_path_endid = new QLabel(page_4);
        label_path_endid->setObjectName(QString::fromUtf8("label_path_endid"));

        gridLayout_4->addWidget(label_path_endid, 4, 1, 1, 1);

        label_path_ID = new QLabel(page_4);
        label_path_ID->setObjectName(QString::fromUtf8("label_path_ID"));

        gridLayout_4->addWidget(label_path_ID, 0, 1, 1, 1);

        verticalSpacer_3 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        gridLayout_4->addItem(verticalSpacer_3, 5, 0, 1, 1);

        label_20 = new QLabel(page_4);
        label_20->setObjectName(QString::fromUtf8("label_20"));

        gridLayout_4->addWidget(label_20, 2, 0, 1, 1);

        comb_direct = new QComboBox(page_4);
        comb_direct->setObjectName(QString::fromUtf8("comb_direct"));

        gridLayout_4->addWidget(comb_direct, 2, 1, 1, 1);

        stackedWidget_Info->addWidget(page_4);

        horizontalLayout->addWidget(stackedWidget_Info);


        gridLayout_2->addLayout(horizontalLayout, 2, 0, 1, 1);

        widget = new QWidget(CAGVWidget);
        widget->setObjectName(QString::fromUtf8("widget"));
        widget->setMaximumSize(QSize(16777215, 100));
        gridLayout = new QGridLayout(widget);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        tabWidget = new QTabWidget(widget);
        tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
        tabWidget->setStyleSheet(QString::fromUtf8(""));
        tab_MapManager = new QWidget();
        tab_MapManager->setObjectName(QString::fromUtf8("tab_MapManager"));
        horizontalLayout_2 = new QHBoxLayout(tab_MapManager);
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        toolButton_scanmap = new QToolButton(tab_MapManager);
        toolButton_scanmap->setObjectName(QString::fromUtf8("toolButton_scanmap"));
        toolButton_scanmap->setStyleSheet(QString::fromUtf8(""));
        toolButton_scanmap->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
        toolButton_scanmap->setAutoRaise(true);
        toolButton_scanmap->setArrowType(Qt::NoArrow);

        horizontalLayout_2->addWidget(toolButton_scanmap);

        toolButton_LoadMap = new QToolButton(tab_MapManager);
        toolButton_LoadMap->setObjectName(QString::fromUtf8("toolButton_LoadMap"));
        toolButton_LoadMap->setStyleSheet(QString::fromUtf8(""));
        toolButton_LoadMap->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
        toolButton_LoadMap->setAutoRaise(true);
        toolButton_LoadMap->setArrowType(Qt::NoArrow);

        horizontalLayout_2->addWidget(toolButton_LoadMap);

        toolButton_UpMap = new QToolButton(tab_MapManager);
        toolButton_UpMap->setObjectName(QString::fromUtf8("toolButton_UpMap"));
        toolButton_UpMap->setEnabled(true);
        toolButton_UpMap->setStyleSheet(QString::fromUtf8(""));
        toolButton_UpMap->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
        toolButton_UpMap->setAutoRaise(true);
        toolButton_UpMap->setArrowType(Qt::NoArrow);

        horizontalLayout_2->addWidget(toolButton_UpMap);

        toolButton_DownMap = new QToolButton(tab_MapManager);
        toolButton_DownMap->setObjectName(QString::fromUtf8("toolButton_DownMap"));
        toolButton_DownMap->setStyleSheet(QString::fromUtf8(""));
        toolButton_DownMap->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
        toolButton_DownMap->setAutoRaise(true);
        toolButton_DownMap->setArrowType(Qt::NoArrow);

        horizontalLayout_2->addWidget(toolButton_DownMap);

        tabWidget->addTab(tab_MapManager, QString());
        tab_AGVControl = new QWidget();
        tab_AGVControl->setObjectName(QString::fromUtf8("tab_AGVControl"));
        horizontalLayout_3 = new QHBoxLayout(tab_AGVControl);
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        toolButton_5 = new QToolButton(tab_AGVControl);
        toolButton_5->setObjectName(QString::fromUtf8("toolButton_5"));
        toolButton_5->setStyleSheet(QString::fromUtf8(""));
        toolButton_5->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
        toolButton_5->setAutoRaise(true);
        toolButton_5->setArrowType(Qt::NoArrow);

        horizontalLayout_3->addWidget(toolButton_5);

        btnGoToStation = new QToolButton(tab_AGVControl);
        btnGoToStation->setObjectName(QString::fromUtf8("btnGoToStation"));
        btnGoToStation->setStyleSheet(QString::fromUtf8(""));
        btnGoToStation->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
        btnGoToStation->setAutoRaise(true);
        btnGoToStation->setArrowType(Qt::NoArrow);

        horizontalLayout_3->addWidget(btnGoToStation);

        toolButton_ManualControl = new QToolButton(tab_AGVControl);
        toolButton_ManualControl->setObjectName(QString::fromUtf8("toolButton_ManualControl"));
        toolButton_ManualControl->setStyleSheet(QString::fromUtf8(""));
        toolButton_ManualControl->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
        toolButton_ManualControl->setAutoRaise(true);
        toolButton_ManualControl->setArrowType(Qt::NoArrow);

        horizontalLayout_3->addWidget(toolButton_ManualControl);

        tabWidget->addTab(tab_AGVControl, QString());
        tab_AGVParam = new QWidget();
        tab_AGVParam->setObjectName(QString::fromUtf8("tab_AGVParam"));
        horizontalLayout_4 = new QHBoxLayout(tab_AGVParam);
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        toolButton_14 = new QToolButton(tab_AGVParam);
        toolButton_14->setObjectName(QString::fromUtf8("toolButton_14"));
        toolButton_14->setStyleSheet(QString::fromUtf8(""));
        toolButton_14->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
        toolButton_14->setAutoRaise(true);
        toolButton_14->setArrowType(Qt::NoArrow);

        horizontalLayout_4->addWidget(toolButton_14);

        tabWidget->addTab(tab_AGVParam, QString());
        tab_MapEdit = new QWidget();
        tab_MapEdit->setObjectName(QString::fromUtf8("tab_MapEdit"));
        horizontalLayout_8 = new QHBoxLayout(tab_MapEdit);
        horizontalLayout_8->setObjectName(QString::fromUtf8("horizontalLayout_8"));
        toolButton_pointer = new QToolButton(tab_MapEdit);
        toolButton_pointer->setObjectName(QString::fromUtf8("toolButton_pointer"));
        toolButton_pointer->setStyleSheet(QString::fromUtf8(""));
        toolButton_pointer->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
        toolButton_pointer->setAutoRaise(true);
        toolButton_pointer->setArrowType(Qt::NoArrow);

        horizontalLayout_8->addWidget(toolButton_pointer);

        toolButton_station = new QToolButton(tab_MapEdit);
        toolButton_station->setObjectName(QString::fromUtf8("toolButton_station"));
        toolButton_station->setStyleSheet(QString::fromUtf8(""));
        toolButton_station->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
        toolButton_station->setAutoRaise(true);
        toolButton_station->setArrowType(Qt::NoArrow);

        horizontalLayout_8->addWidget(toolButton_station);

        toolButton_line = new QToolButton(tab_MapEdit);
        toolButton_line->setObjectName(QString::fromUtf8("toolButton_line"));
        toolButton_line->setStyleSheet(QString::fromUtf8(""));
        toolButton_line->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
        toolButton_line->setAutoRaise(true);
        toolButton_line->setArrowType(Qt::NoArrow);

        horizontalLayout_8->addWidget(toolButton_line);

        toolButton_bezier = new QToolButton(tab_MapEdit);
        toolButton_bezier->setObjectName(QString::fromUtf8("toolButton_bezier"));
        toolButton_bezier->setStyleSheet(QString::fromUtf8(""));
        toolButton_bezier->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
        toolButton_bezier->setAutoRaise(true);
        toolButton_bezier->setArrowType(Qt::NoArrow);

        horizontalLayout_8->addWidget(toolButton_bezier);

        toolButton_eraser = new QToolButton(tab_MapEdit);
        toolButton_eraser->setObjectName(QString::fromUtf8("toolButton_eraser"));
        toolButton_eraser->setStyleSheet(QString::fromUtf8(""));
        toolButton_eraser->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
        toolButton_eraser->setAutoRaise(true);
        toolButton_eraser->setArrowType(Qt::NoArrow);

        horizontalLayout_8->addWidget(toolButton_eraser);

        line = new QFrame(tab_MapEdit);
        line->setObjectName(QString::fromUtf8("line"));
        line->setFrameShape(QFrame::VLine);
        line->setFrameShadow(QFrame::Sunken);

        horizontalLayout_8->addWidget(line);

        toolButton_Del = new QToolButton(tab_MapEdit);
        toolButton_Del->setObjectName(QString::fromUtf8("toolButton_Del"));
        toolButton_Del->setStyleSheet(QString::fromUtf8(""));
        toolButton_Del->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
        toolButton_Del->setAutoRaise(true);
        toolButton_Del->setArrowType(Qt::NoArrow);

        horizontalLayout_8->addWidget(toolButton_Del);

        toolButton_SaveMap = new QToolButton(tab_MapEdit);
        toolButton_SaveMap->setObjectName(QString::fromUtf8("toolButton_SaveMap"));
        toolButton_SaveMap->setStyleSheet(QString::fromUtf8(""));
        toolButton_SaveMap->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
        toolButton_SaveMap->setAutoRaise(true);
        toolButton_SaveMap->setArrowType(Qt::NoArrow);

        horizontalLayout_8->addWidget(toolButton_SaveMap);

        toolButton_map_check = new QToolButton(tab_MapEdit);
        toolButton_map_check->setObjectName(QString::fromUtf8("toolButton_map_check"));
        toolButton_map_check->setStyleSheet(QString::fromUtf8(""));
        toolButton_map_check->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
        toolButton_map_check->setAutoRaise(true);
        toolButton_map_check->setArrowType(Qt::NoArrow);

        horizontalLayout_8->addWidget(toolButton_map_check);

        line_3 = new QFrame(tab_MapEdit);
        line_3->setObjectName(QString::fromUtf8("line_3"));
        line_3->setFrameShape(QFrame::VLine);
        line_3->setFrameShadow(QFrame::Sunken);

        horizontalLayout_8->addWidget(line_3);

        toolButton_undo = new QToolButton(tab_MapEdit);
        toolButton_undo->setObjectName(QString::fromUtf8("toolButton_undo"));
        toolButton_undo->setStyleSheet(QString::fromUtf8(""));
        toolButton_undo->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
        toolButton_undo->setAutoRaise(true);
        toolButton_undo->setArrowType(Qt::NoArrow);

        horizontalLayout_8->addWidget(toolButton_undo);

        toolButton_redo = new QToolButton(tab_MapEdit);
        toolButton_redo->setObjectName(QString::fromUtf8("toolButton_redo"));
        toolButton_redo->setStyleSheet(QString::fromUtf8(""));
        toolButton_redo->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
        toolButton_redo->setAutoRaise(true);
        toolButton_redo->setArrowType(Qt::NoArrow);

        horizontalLayout_8->addWidget(toolButton_redo);

        tabWidget->addTab(tab_MapEdit, QString());

        gridLayout->addWidget(tabWidget, 0, 1, 1, 1);

        btn_BackHome = new QToolButton(widget);
        btn_BackHome->setObjectName(QString::fromUtf8("btn_BackHome"));
        btn_BackHome->setMinimumSize(QSize(0, 50));

        gridLayout->addWidget(btn_BackHome, 0, 0, 1, 1);


        gridLayout_2->addWidget(widget, 0, 0, 1, 1);

        stackedWidget = new QStackedWidget(CAGVWidget);
        stackedWidget->setObjectName(QString::fromUtf8("stackedWidget"));
        stackedWidget->setMaximumSize(QSize(16777215, 50));
        page = new QWidget();
        page->setObjectName(QString::fromUtf8("page"));
        horizontalLayout_6 = new QHBoxLayout(page);
        horizontalLayout_6->setObjectName(QString::fromUtf8("horizontalLayout_6"));
        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
        label = new QLabel(page);
        label->setObjectName(QString::fromUtf8("label"));

        horizontalLayout_5->addWidget(label);

        EraserSlider = new QSlider(page);
        EraserSlider->setObjectName(QString::fromUtf8("EraserSlider"));
        EraserSlider->setMaximumSize(QSize(200, 16777215));
        EraserSlider->setMinimum(10);
        EraserSlider->setMaximum(110);
        EraserSlider->setOrientation(Qt::Horizontal);

        horizontalLayout_5->addWidget(EraserSlider);

        pushButton = new QPushButton(page);
        pushButton->setObjectName(QString::fromUtf8("pushButton"));

        horizontalLayout_5->addWidget(pushButton);

        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_5->addItem(horizontalSpacer_2);


        horizontalLayout_6->addLayout(horizontalLayout_5);

        stackedWidget->addWidget(page);
        page_2 = new QWidget();
        page_2->setObjectName(QString::fromUtf8("page_2"));
        horizontalLayout_9 = new QHBoxLayout(page_2);
        horizontalLayout_9->setObjectName(QString::fromUtf8("horizontalLayout_9"));
        horizontalLayout_7 = new QHBoxLayout();
        horizontalLayout_7->setObjectName(QString::fromUtf8("horizontalLayout_7"));
        pushButton_add_station = new QPushButton(page_2);
        pushButton_add_station->setObjectName(QString::fromUtf8("pushButton_add_station"));

        horizontalLayout_7->addWidget(pushButton_add_station);

        pushButton_add_station_agv = new QPushButton(page_2);
        pushButton_add_station_agv->setObjectName(QString::fromUtf8("pushButton_add_station_agv"));

        horizontalLayout_7->addWidget(pushButton_add_station_agv);

        pushButton_mov_stationtoagv = new QPushButton(page_2);
        pushButton_mov_stationtoagv->setObjectName(QString::fromUtf8("pushButton_mov_stationtoagv"));

        horizontalLayout_7->addWidget(pushButton_mov_stationtoagv);

        horizontalSpacer_3 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_7->addItem(horizontalSpacer_3);


        horizontalLayout_9->addLayout(horizontalLayout_7);

        stackedWidget->addWidget(page_2);

        gridLayout_2->addWidget(stackedWidget, 1, 0, 1, 1);


        retranslateUi(CAGVWidget);

        stackedWidget_Info->setCurrentIndex(2);
        tabWidget->setCurrentIndex(0);
        stackedWidget->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(CAGVWidget);
    } // setupUi

    void retranslateUi(QWidget *CAGVWidget)
    {
        CAGVWidget->setWindowTitle(QApplication::translate("CAGVWidget", "Form", nullptr));
        label_2->setText(QApplication::translate("CAGVWidget", "\345\234\260\345\233\276\345\256\275\357\274\232", nullptr));
        label_map_width->setText(QApplication::translate("CAGVWidget", "\345\234\260\345\233\276\345\256\275:", nullptr));
        label_map_name->setText(QApplication::translate("CAGVWidget", "\345\234\260\345\233\276\345\220\215\347\247\260\357\274\232", nullptr));
        label_map_height->setText(QApplication::translate("CAGVWidget", "\345\234\260\345\233\276\351\253\230:", nullptr));
        label_4->setText(QApplication::translate("CAGVWidget", "\345\234\260\345\233\276\345\220\215\347\247\260\357\274\232", nullptr));
        label_1->setText(QApplication::translate("CAGVWidget", "\345\234\260\345\233\276\345\210\206\350\276\250\347\216\207\357\274\232", nullptr));
        label_map_resolution->setText(QApplication::translate("CAGVWidget", "\345\234\260\345\233\276\345\210\206\350\276\250\347\216\207\357\274\232", nullptr));
        label_3->setText(QApplication::translate("CAGVWidget", "\345\234\260\345\233\276\351\253\230\357\274\232", nullptr));
        label_st_type->setText(QApplication::translate("CAGVWidget", " \347\253\231\347\202\271\347\261\273\345\236\213\357\274\232", nullptr));
        label_st_worldx->setText(QApplication::translate("CAGVWidget", "x\357\274\232", nullptr));
        label_st_angle->setText(QApplication::translate("CAGVWidget", "\346\234\235\345\220\221\350\247\222\357\274\232", nullptr));
        label_7->setText(QApplication::translate("CAGVWidget", "grid_x\357\274\232", nullptr));
        label_9->setText(QApplication::translate("CAGVWidget", "grid_y\357\274\232", nullptr));
        label_8->setText(QApplication::translate("CAGVWidget", "world_x\357\274\232", nullptr));
        label_5->setText(QApplication::translate("CAGVWidget", " \347\253\231\347\202\271ID\357\274\232", nullptr));
        label_11->setText(QApplication::translate("CAGVWidget", "\346\234\235\345\220\221\350\247\222\357\274\232", nullptr));
        label_st_id->setText(QApplication::translate("CAGVWidget", " \347\253\231\347\202\271ID\357\274\232", nullptr));
        label_14->setText(QApplication::translate("CAGVWidget", " \347\253\231\347\202\271\347\261\273\345\236\213\357\274\232", nullptr));
        label_st_x->setText(QApplication::translate("CAGVWidget", "x\357\274\232", nullptr));
        label_st_y->setText(QApplication::translate("CAGVWidget", "y\357\274\232", nullptr));
        label_10->setText(QApplication::translate("CAGVWidget", "world_y\357\274\232", nullptr));
        label_st_worldy->setText(QApplication::translate("CAGVWidget", "x\357\274\232", nullptr));
        label_13->setText(QApplication::translate("CAGVWidget", "\350\267\257\347\272\277ID\357\274\232", nullptr));
        label_path_startid->setText(QApplication::translate("CAGVWidget", "\350\265\267\345\247\213\347\202\271ID\357\274\232", nullptr));
        label_17->setText(QApplication::translate("CAGVWidget", "\347\273\210\346\255\242\347\202\271ID\357\274\232", nullptr));
        label_15->setText(QApplication::translate("CAGVWidget", "\350\265\267\345\247\213\347\202\271ID\357\274\232", nullptr));
        label_path_type->setText(QApplication::translate("CAGVWidget", "\350\267\257\347\272\277\347\261\273\345\236\213\357\274\232", nullptr));
        label_19->setText(QApplication::translate("CAGVWidget", "\350\267\257\347\272\277\347\261\273\345\236\213\357\274\232", nullptr));
        label_path_endid->setText(QApplication::translate("CAGVWidget", "\347\273\210\346\255\242\347\202\271ID\357\274\232", nullptr));
        label_path_ID->setText(QApplication::translate("CAGVWidget", "\350\267\257\347\272\277ID\357\274\232", nullptr));
        label_20->setText(QApplication::translate("CAGVWidget", "\350\267\257\347\272\277\346\226\271\345\220\221\357\274\232", nullptr));
        toolButton_scanmap->setText(QApplication::translate("CAGVWidget", "  \346\236\204\345\273\272\345\234\260\345\233\276", nullptr));
        toolButton_LoadMap->setText(QApplication::translate("CAGVWidget", "\345\212\240\350\275\275\345\234\260\345\233\276", nullptr));
        toolButton_UpMap->setText(QApplication::translate("CAGVWidget", "  \344\270\212\344\274\240\345\234\260\345\233\276", nullptr));
        toolButton_DownMap->setText(QApplication::translate("CAGVWidget", "\344\270\213\350\275\275\345\234\260\345\233\276", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_MapManager), QApplication::translate("CAGVWidget", " \345\234\260\345\233\276\347\256\241\347\220\206", nullptr));
        toolButton_5->setText(QApplication::translate("CAGVWidget", " \351\207\215\345\256\232\344\275\215", nullptr));
        btnGoToStation->setText(QApplication::translate("CAGVWidget", "\347\253\231\347\202\271\345\257\274\350\210\252", nullptr));
        toolButton_ManualControl->setText(QApplication::translate("CAGVWidget", "\346\211\213\345\212\250\346\216\247\345\210\266", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_AGVControl), QApplication::translate("CAGVWidget", "AGV\346\216\247\345\210\266", nullptr));
        toolButton_14->setText(QApplication::translate("CAGVWidget", "AGV\345\217\202\346\225\260\351\205\215\347\275\256", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_AGVParam), QApplication::translate("CAGVWidget", " \345\217\202\346\225\260\351\205\215\347\275\256", nullptr));
        toolButton_pointer->setText(QApplication::translate("CAGVWidget", " <\347\256\255\345\244\264>", nullptr));
        toolButton_station->setText(QApplication::translate("CAGVWidget", " \347\253\231\347\202\271", nullptr));
        toolButton_line->setText(QApplication::translate("CAGVWidget", "\347\233\264\347\272\277", nullptr));
        toolButton_bezier->setText(QApplication::translate("CAGVWidget", "   \350\264\235\345\241\236\345\260\224\346\233\262\347\272\277", nullptr));
        toolButton_eraser->setText(QApplication::translate("CAGVWidget", " \346\251\241\347\232\256\346\223\246", nullptr));
        toolButton_Del->setText(QApplication::translate("CAGVWidget", "\345\210\240\351\231\244", nullptr));
        toolButton_SaveMap->setText(QApplication::translate("CAGVWidget", "\344\277\235\345\255\230\345\234\260\345\233\276", nullptr));
        toolButton_map_check->setText(QApplication::translate("CAGVWidget", " \345\234\260\345\233\276\346\243\200\346\237\245", nullptr));
        toolButton_undo->setText(QApplication::translate("CAGVWidget", " \346\222\244\351\224\200", nullptr));
        toolButton_redo->setText(QApplication::translate("CAGVWidget", "\346\201\242\345\244\215", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_MapEdit), QApplication::translate("CAGVWidget", " \345\234\260\345\233\276\347\274\226\350\276\221", nullptr));
        btn_BackHome->setText(QApplication::translate("CAGVWidget", "\350\277\224\345\233\236\351\246\226\351\241\265", nullptr));
        label->setText(QApplication::translate("CAGVWidget", "    \346\251\241\347\232\256\346\223\246:", nullptr));
        pushButton->setText(QApplication::translate("CAGVWidget", "  \346\270\205\351\231\244\345\255\244\347\253\213\347\202\271", nullptr));
        pushButton_add_station->setText(QApplication::translate("CAGVWidget", " \346\267\273\345\212\240\347\253\231\347\202\271", nullptr));
        pushButton_add_station_agv->setText(QApplication::translate("CAGVWidget", "\345\234\250AGV\345\244\204\346\267\273\345\212\240\347\253\231\347\202\271", nullptr));
        pushButton_mov_stationtoagv->setText(QApplication::translate("CAGVWidget", "\347\247\273\345\212\250\347\253\231\347\202\271\345\210\260AGV\345\244\204", nullptr));
    } // retranslateUi

};

namespace Ui {
    class CAGVWidget: public Ui_CAGVWidget {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_AGVWIDGET_H
