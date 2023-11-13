/********************************************************************************
** Form generated from reading UI file 'AGVListWindow.ui'
**
** Created by: Qt User Interface Compiler version 5.12.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_AGVLISTWINDOW_H
#define UI_AGVLISTWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QTableWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_CAGVListWindow
{
public:
    QHBoxLayout *horizontalLayout_4;
    QGridLayout *gridLayout_12;
    QPushButton *pushButton_Delete;
    QSpacerItem *horizontalSpacer_22;
    QPushButton *pushButton_ScanAGV;
    QTableWidget *tableAGVs;
    QSpacerItem *horizontalSpacer_21;
    QVBoxLayout *verticalLayout_3;
    QSpacerItem *verticalSpacer_13;
    QWidget *widget_WIFI;
    QGridLayout *gridLayout_9;
    QLabel *label_25;
    QTableWidget *tableWidget_WIFI;
    QSpacerItem *verticalSpacer_14;
    QWidget *widget_AGVControl;
    QGridLayout *gridLayout_10;
    QGridLayout *gridLayout_11;
    QLabel *label_30;
    QLabel *label_31;
    QLabel *label_CPU;
    QLabel *label_33;
    QLabel *label_Ver;
    QLabel *label_MEM;
    QSpacerItem *verticalSpacer_15;
    QHBoxLayout *horizontalLayout_3;
    QLabel *label_36;
    QSpacerItem *horizontalSpacer_18;
    QSpacerItem *verticalSpacer_16;
    QSpacerItem *horizontalSpacer_19;
    QSpacerItem *verticalSpacer_17;
    QSpacerItem *horizontalSpacer_20;
    QSpacerItem *verticalSpacer_18;

    void setupUi(QWidget *CAGVListWindow)
    {
        if (CAGVListWindow->objectName().isEmpty())
            CAGVListWindow->setObjectName(QString::fromUtf8("CAGVListWindow"));
        CAGVListWindow->resize(1187, 636);
        horizontalLayout_4 = new QHBoxLayout(CAGVListWindow);
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        gridLayout_12 = new QGridLayout();
        gridLayout_12->setObjectName(QString::fromUtf8("gridLayout_12"));
        pushButton_Delete = new QPushButton(CAGVListWindow);
        pushButton_Delete->setObjectName(QString::fromUtf8("pushButton_Delete"));

        gridLayout_12->addWidget(pushButton_Delete, 0, 0, 1, 1);

        horizontalSpacer_22 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        gridLayout_12->addItem(horizontalSpacer_22, 0, 1, 1, 1);

        pushButton_ScanAGV = new QPushButton(CAGVListWindow);
        pushButton_ScanAGV->setObjectName(QString::fromUtf8("pushButton_ScanAGV"));

        gridLayout_12->addWidget(pushButton_ScanAGV, 0, 2, 1, 1);

        tableAGVs = new QTableWidget(CAGVListWindow);
        tableAGVs->setObjectName(QString::fromUtf8("tableAGVs"));

        gridLayout_12->addWidget(tableAGVs, 1, 0, 1, 3);


        horizontalLayout_4->addLayout(gridLayout_12);

        horizontalSpacer_21 = new QSpacerItem(20, 20, QSizePolicy::Fixed, QSizePolicy::Minimum);

        horizontalLayout_4->addItem(horizontalSpacer_21);

        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        verticalSpacer_13 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_3->addItem(verticalSpacer_13);

        widget_WIFI = new QWidget(CAGVListWindow);
        widget_WIFI->setObjectName(QString::fromUtf8("widget_WIFI"));
        widget_WIFI->setMaximumSize(QSize(500, 16777215));
        gridLayout_9 = new QGridLayout(widget_WIFI);
        gridLayout_9->setObjectName(QString::fromUtf8("gridLayout_9"));
        label_25 = new QLabel(widget_WIFI);
        label_25->setObjectName(QString::fromUtf8("label_25"));

        gridLayout_9->addWidget(label_25, 0, 0, 1, 1);

        tableWidget_WIFI = new QTableWidget(widget_WIFI);
        tableWidget_WIFI->setObjectName(QString::fromUtf8("tableWidget_WIFI"));
        tableWidget_WIFI->setMaximumSize(QSize(400, 16777215));

        gridLayout_9->addWidget(tableWidget_WIFI, 1, 0, 2, 1);


        verticalLayout_3->addWidget(widget_WIFI);

        verticalSpacer_14 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Fixed);

        verticalLayout_3->addItem(verticalSpacer_14);

        widget_AGVControl = new QWidget(CAGVListWindow);
        widget_AGVControl->setObjectName(QString::fromUtf8("widget_AGVControl"));
        widget_AGVControl->setMaximumSize(QSize(500, 16777215));
        gridLayout_10 = new QGridLayout(widget_AGVControl);
        gridLayout_10->setObjectName(QString::fromUtf8("gridLayout_10"));
        gridLayout_11 = new QGridLayout();
        gridLayout_11->setObjectName(QString::fromUtf8("gridLayout_11"));
        label_30 = new QLabel(widget_AGVControl);
        label_30->setObjectName(QString::fromUtf8("label_30"));
        label_30->setMaximumSize(QSize(200, 16777215));

        gridLayout_11->addWidget(label_30, 0, 0, 1, 1);

        label_31 = new QLabel(widget_AGVControl);
        label_31->setObjectName(QString::fromUtf8("label_31"));
        label_31->setMaximumSize(QSize(200, 16777215));

        gridLayout_11->addWidget(label_31, 1, 0, 1, 1);

        label_CPU = new QLabel(widget_AGVControl);
        label_CPU->setObjectName(QString::fromUtf8("label_CPU"));
        label_CPU->setMaximumSize(QSize(200, 16777215));

        gridLayout_11->addWidget(label_CPU, 1, 1, 1, 1);

        label_33 = new QLabel(widget_AGVControl);
        label_33->setObjectName(QString::fromUtf8("label_33"));
        label_33->setMaximumSize(QSize(200, 16777215));

        gridLayout_11->addWidget(label_33, 2, 0, 1, 1);

        label_Ver = new QLabel(widget_AGVControl);
        label_Ver->setObjectName(QString::fromUtf8("label_Ver"));
        label_Ver->setMaximumSize(QSize(200, 16777215));

        gridLayout_11->addWidget(label_Ver, 2, 1, 1, 1);

        label_MEM = new QLabel(widget_AGVControl);
        label_MEM->setObjectName(QString::fromUtf8("label_MEM"));
        label_MEM->setMaximumSize(QSize(200, 16777215));

        gridLayout_11->addWidget(label_MEM, 0, 1, 1, 1);


        gridLayout_10->addLayout(gridLayout_11, 3, 1, 1, 1);

        verticalSpacer_15 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Fixed);

        gridLayout_10->addItem(verticalSpacer_15, 0, 1, 1, 1);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        label_36 = new QLabel(widget_AGVControl);
        label_36->setObjectName(QString::fromUtf8("label_36"));

        horizontalLayout_3->addWidget(label_36);

        horizontalSpacer_18 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_3->addItem(horizontalSpacer_18);


        gridLayout_10->addLayout(horizontalLayout_3, 1, 1, 1, 1);

        verticalSpacer_16 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Fixed);

        gridLayout_10->addItem(verticalSpacer_16, 4, 1, 1, 1);

        horizontalSpacer_19 = new QSpacerItem(40, 20, QSizePolicy::Fixed, QSizePolicy::Minimum);

        gridLayout_10->addItem(horizontalSpacer_19, 3, 0, 1, 1);

        verticalSpacer_17 = new QSpacerItem(20, 10, QSizePolicy::Minimum, QSizePolicy::Fixed);

        gridLayout_10->addItem(verticalSpacer_17, 2, 1, 1, 1);

        horizontalSpacer_20 = new QSpacerItem(40, 20, QSizePolicy::Fixed, QSizePolicy::Minimum);

        gridLayout_10->addItem(horizontalSpacer_20, 3, 2, 1, 1);


        verticalLayout_3->addWidget(widget_AGVControl);

        verticalSpacer_18 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_3->addItem(verticalSpacer_18);


        horizontalLayout_4->addLayout(verticalLayout_3);


        retranslateUi(CAGVListWindow);

        QMetaObject::connectSlotsByName(CAGVListWindow);
    } // setupUi

    void retranslateUi(QWidget *CAGVListWindow)
    {
        CAGVListWindow->setWindowTitle(QApplication::translate("CAGVListWindow", "Form", nullptr));
        pushButton_Delete->setText(QApplication::translate("CAGVListWindow", "\345\210\240\351\231\244", nullptr));
        pushButton_ScanAGV->setText(QApplication::translate("CAGVListWindow", "\346\211\253\346\217\217", nullptr));
        label_25->setText(QApplication::translate("CAGVListWindow", "WIFI\345\210\227\350\241\250", nullptr));
        label_30->setText(QApplication::translate("CAGVListWindow", "\345\206\205\345\255\230\345\215\240\347\224\250\357\274\232", nullptr));
        label_31->setText(QApplication::translate("CAGVListWindow", "CPU\345\215\240\347\224\250\357\274\232", nullptr));
        label_CPU->setText(QApplication::translate("CAGVListWindow", "10%", nullptr));
        label_33->setText(QApplication::translate("CAGVListWindow", "\347\211\210\346\234\254\345\217\267\357\274\232", nullptr));
        label_Ver->setText(QApplication::translate("CAGVListWindow", "1.0.0", nullptr));
        label_MEM->setText(QApplication::translate("CAGVListWindow", "80%", nullptr));
        label_36->setText(QApplication::translate("CAGVListWindow", "AGV\346\216\247\345\210\266\345\231\250\344\277\241\346\201\257", nullptr));
    } // retranslateUi

};

namespace Ui {
    class CAGVListWindow: public Ui_CAGVListWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_AGVLISTWINDOW_H
