/********************************************************************************
** Form generated from reading UI file 'CoordinateWidget.ui'
**
** Created by: Qt User Interface Compiler version 5.12.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_COORDINATEWIDGET_H
#define UI_COORDINATEWIDGET_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_CCoordinateWidget
{
public:
    QGridLayout *gridLayout;
    QLabel *label;
    QLabel *label_wordx;
    QLabel *label_4;
    QLabel *label_worldy;
    QLabel *label_8;
    QLabel *label_mapx;
    QLabel *label_6;
    QLabel *label_mapy;

    void setupUi(QWidget *CCoordinateWidget)
    {
        if (CCoordinateWidget->objectName().isEmpty())
            CCoordinateWidget->setObjectName(QString::fromUtf8("CCoordinateWidget"));
        CCoordinateWidget->resize(155, 53);
        CCoordinateWidget->setMaximumSize(QSize(250, 16777215));
        CCoordinateWidget->setWindowOpacity(0.000000000000000);
        gridLayout = new QGridLayout(CCoordinateWidget);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        label = new QLabel(CCoordinateWidget);
        label->setObjectName(QString::fromUtf8("label"));
        label->setMaximumSize(QSize(40, 16777215));

        gridLayout->addWidget(label, 0, 0, 1, 1);

        label_wordx = new QLabel(CCoordinateWidget);
        label_wordx->setObjectName(QString::fromUtf8("label_wordx"));
        label_wordx->setMaximumSize(QSize(100, 16777215));

        gridLayout->addWidget(label_wordx, 0, 1, 1, 1);

        label_4 = new QLabel(CCoordinateWidget);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setMaximumSize(QSize(40, 16777215));

        gridLayout->addWidget(label_4, 0, 2, 1, 1);

        label_worldy = new QLabel(CCoordinateWidget);
        label_worldy->setObjectName(QString::fromUtf8("label_worldy"));

        gridLayout->addWidget(label_worldy, 0, 3, 1, 1);

        label_8 = new QLabel(CCoordinateWidget);
        label_8->setObjectName(QString::fromUtf8("label_8"));

        gridLayout->addWidget(label_8, 1, 0, 1, 1);

        label_mapx = new QLabel(CCoordinateWidget);
        label_mapx->setObjectName(QString::fromUtf8("label_mapx"));

        gridLayout->addWidget(label_mapx, 1, 1, 1, 1);

        label_6 = new QLabel(CCoordinateWidget);
        label_6->setObjectName(QString::fromUtf8("label_6"));

        gridLayout->addWidget(label_6, 1, 2, 1, 1);

        label_mapy = new QLabel(CCoordinateWidget);
        label_mapy->setObjectName(QString::fromUtf8("label_mapy"));

        gridLayout->addWidget(label_mapy, 1, 3, 1, 1);


        retranslateUi(CCoordinateWidget);

        QMetaObject::connectSlotsByName(CCoordinateWidget);
    } // setupUi

    void retranslateUi(QWidget *CCoordinateWidget)
    {
        CCoordinateWidget->setWindowTitle(QApplication::translate("CCoordinateWidget", "Form", nullptr));
        label->setText(QApplication::translate("CCoordinateWidget", "wordx:", nullptr));
        label_wordx->setText(QApplication::translate("CCoordinateWidget", "100", nullptr));
        label_4->setText(QApplication::translate("CCoordinateWidget", "wordy:", nullptr));
        label_worldy->setText(QApplication::translate("CCoordinateWidget", "100", nullptr));
        label_8->setText(QApplication::translate("CCoordinateWidget", "mapx:", nullptr));
        label_mapx->setText(QApplication::translate("CCoordinateWidget", "100", nullptr));
        label_6->setText(QApplication::translate("CCoordinateWidget", "mapy:", nullptr));
        label_mapy->setText(QApplication::translate("CCoordinateWidget", "100", nullptr));
    } // retranslateUi

};

namespace Ui {
    class CCoordinateWidget: public Ui_CCoordinateWidget {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_COORDINATEWIDGET_H
