//创建各种光标
#ifndef CCURSORFACTORY_H
#define CCURSORFACTORY_H
#include <QCursor>
#include <QPixmap>

enum class ECursor{
    Cursor_Station,  //站点
    Cursor_Line,     //路线
    Cursor_Eraser,   //橡皮擦
    Cursor_Des,      //目标站点
};

class CCursorFactory
{
public:
    CCursorFactory();

    //创建其他光标
    QCursor CreateCursor(ECursor eCursor);

private:
    QCursor CreateCursorPrivate(QPixmap pixmap, int iOpacity);
    QCursor m_station;
    QCursor m_line;
    QCursor m_eraser;
    QCursor m_des; //目标站

};

#endif // CCURSORFACTORY_H
