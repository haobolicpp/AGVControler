#ifndef MAPVIEW_DEF_H
#define MAPVIEW_DEF_H
#include <QGraphicsItem>

//自定义Item的Type
#define ITEM_ERASER QGraphicsItem::UserType + 1
#define ITEM_LINE QGraphicsItem::UserType + 2
#define ITEM_MAP QGraphicsItem::UserType + 3
#define ITEM_STATION QGraphicsItem::UserType + 4
#define ITEM_STATION_TXT QGraphicsItem::UserType + 5
#define ITEM_AGV QGraphicsItem::UserType + 6
#define ITEM_GLOBALPATH QGraphicsItem::UserType + 7

//线控制点定义
#define LINE_CTRLPOINT_LEN  4  //控制点大小
//点到直线的距离(像素值) 当小于该值时自动吸附到线上
#define POINTTOLINE_DISTANCE_MIN   6.0

//孤立点距离（两点之间距离大于改值则为孤立点）
#define CLEAROUTLIER 6

//未选中时线宽
#define LINE_WIDTH_NO_SELECTED 2
//hover进入或选中时线宽
#define LINE_WIDTH_SELECTED_HOVER 3

//线上箭头的长度 采样间隔个数
#define LINE_ARROW_LENGTH 200
//线上箭头的宽度(像素)
#define LINE_ARROW_WIDTH 5

//站点初始角度
#define STATTION_INIT_ANGLE -90
//站点颜色定义
#define GENERAL_COLOR QColor(128, 140, 255)
#define GENERAL_COLOR_SELECTED QColor(83,84,154)
#define CHARGE_COLOR QColor(212,254,0)
#define CHARGE_COLOR_SELECTED QColor(164,197,0)
//站点长宽定义
#define STATION_WIDTH 20  //宽(像素)
#define STATION_HIGHT 25  //高
#define STATION_CONTROL_LEN 4 //控制点
//缩小站点为圆形定义
#define SMALL_STATION_LEN 10 //半径
#define SMALL_

//AGV绘制的初始角度
#define AGV_INIT_ANGLE (-90)
//AGV长宽定义
#define AGV_WIDTH 15  //宽(像素)
#define AGV_HIGHT 20  //高
#define AGV_COLOR QColor(128, 200, 200)
#define AGV_COLOR_SELECTED QColor(83,84,154)
#define AGV_COLOR_WHEEL QColor(100, 200, 20)

//View中World原点坐标轴
#define AXIS_LEN 30
#define AXIS_POINTER_LEN 5

//当前鼠标状态
enum class EMouseMode{
    MouseMode_Pointer = 0x1000,   //指针
    MouseMode_Eraser = 0x2000,    //橡皮擦
    MouseMode_Station = 0x3000,   //站点

    //连接线
    MouseMode_Line = 0x4000,   //直线
    MouseMode_Arc = 0x4001,   //圆弧
    MouseMode_Bezier = 0x4002,   //贝塞尔曲线

    //其他
    MouseMode_ToDes = 0x5000, //目标站点导航

    //区域


};

#endif // MAPVIEW_DEF_H
