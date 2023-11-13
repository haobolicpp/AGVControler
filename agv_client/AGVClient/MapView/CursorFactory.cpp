#include <QPixmap>
#include <QPainter>
#include "CursorFactory.h"

CCursorFactory::CCursorFactory()
{
    m_station = CreateCursorPrivate(QPixmap(":/Image/resource/cursor/cursor_station.png"),150);
    m_line = CreateCursorPrivate(QPixmap(":/Image/resource/cursor/cursor_line.png"),150);
    m_eraser = CreateCursorPrivate(QPixmap(":/Image/resource/cursor/cursor_line.png"),0); //橡皮擦鼠标完全透明
    m_des = CreateCursorPrivate(QPixmap(":/Image/resource/cursor/cursor_des.png"), 150);
}

QCursor CCursorFactory::CreateCursor(ECursor eCursor)
{
    if (eCursor == ECursor::Cursor_Station){
        return m_station;
    }else if(eCursor == ECursor::Cursor_Line){
        return m_line;
    }
    else if (eCursor == ECursor::Cursor_Des)
    {
        return m_des;
    }
    else if (eCursor == ECursor::Cursor_Eraser)
    {
        return m_eraser;
    }
    else
    {

    }
    return QCursor();
}

QCursor CCursorFactory::CreateCursorPrivate(QPixmap pixmap, int iOpacity)
{
    //绘制透明pixmap，参考百度搜索“qpainter CompositionMode”
    QPixmap pixmap1 = pixmap;
    QPixmap pixmap2(pixmap1.size());
    pixmap2.fill(Qt::transparent);
    QPainter painter(&pixmap2);
    //将原始图片画进去
    painter.setCompositionMode(QPainter::CompositionMode_Source);
    painter.drawPixmap(0, 0, pixmap1);
    //设置Destination图片，有透明度，最终结果混合后图像具有透明度
    painter.setCompositionMode(QPainter::CompositionMode_DestinationIn);
    painter.fillRect(pixmap2.rect(), QColor(0,0,0,iOpacity));
    painter.end();
    return QCursor(pixmap2);
}

