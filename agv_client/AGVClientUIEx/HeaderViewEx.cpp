#include "HeaderViewEx.h"

CHeaderViewEx::CHeaderViewEx(Qt::Orientation orientation, QWidget *parent)
    :QHeaderView(orientation, parent)
{
    m_pcheckBox = new QCheckBox(this);
    m_pcheckBox->setText(QStringLiteral("全选"));
    connect(m_pcheckBox, &QCheckBox::stateChanged, [&](){
        emit this->SigCheckboxClick(m_pcheckBox->checkState());
    });
}

void CHeaderViewEx::paintSection(QPainter *painter, const QRect &rect, int logicalIndex) const
{
    if (0 == logicalIndex){
        //第0个项
        m_pcheckBox->setGeometry(rect);
    }
    else{
        QHeaderView::paintSection(painter, rect, logicalIndex);
    }
}
