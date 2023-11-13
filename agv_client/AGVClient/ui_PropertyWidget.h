/********************************************************************************
** Form generated from reading UI file 'PropertyWidget.ui'
**
** Created by: Qt User Interface Compiler version 5.12.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_PROPERTYWIDGET_H
#define UI_PROPERTYWIDGET_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_CPropertyWidget
{
public:
    QGridLayout *gridLayout;

    void setupUi(QWidget *CPropertyWidget)
    {
        if (CPropertyWidget->objectName().isEmpty())
            CPropertyWidget->setObjectName(QString::fromUtf8("CPropertyWidget"));
        CPropertyWidget->resize(400, 300);
        gridLayout = new QGridLayout(CPropertyWidget);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));

        retranslateUi(CPropertyWidget);

        QMetaObject::connectSlotsByName(CPropertyWidget);
    } // setupUi

    void retranslateUi(QWidget *CPropertyWidget)
    {
        CPropertyWidget->setWindowTitle(QApplication::translate("CPropertyWidget", "Form", nullptr));
    } // retranslateUi

};

namespace Ui {
    class CPropertyWidget: public Ui_CPropertyWidget {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PROPERTYWIDGET_H
