#ifndef CHEADERVIEWEX_H
#define CHEADERVIEWEX_H
#include <QHeaderView>
#include <QCheckBox>
#include "AGVClientUIEx_global.h"

class AGVCLIENTUIEX_EXPORT CHeaderViewEx : public QHeaderView
{
    Q_OBJECT
public:
    CHeaderViewEx(Qt::Orientation orientation, QWidget *parent = nullptr);

protected:
    void paintSection(QPainter *painter, const QRect &rect, int logicalIndex) const override;

Q_SIGNALS:
    void SigCheckboxClick(Qt::CheckState state);

private:
    QCheckBox *m_pcheckBox;//注意必须是指针，否则paintSection()const中无法操作
};

#endif // CHEADERVIEWEX_H
