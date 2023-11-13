#ifndef CGEOMETRYALGORITHM_H
#define CGEOMETRYALGORITHM_H

#include "gr_type.h"
#include "commonlibrary_global.h"

//浮点型是否是零
#define DOUBLE_IS_ZERO(d) (d>-EPSILON && d<EPSILON)

//坐标
typedef struct TPointd{
    double dX; //X坐标
    double dY; //Y坐标
}TPointd;
typedef struct TPoint{
    int iX; //X坐标
    int iY; //Y坐标
}TPoint;


class COMMONLIBRARYSHARED_EXPORT CGeometryAlgorithm
{
public:
    CGeometryAlgorithm();

    //度和弧度转换
    static double DegToRad(double deg);
    static double RadToDeg(double rad);

    // 计算两个点相对x轴  夹角（2相对1），return（0~180 0~-180°）
    static double Point2PointAngle(double dx1, double dy1, double dx2, double dy2);

    /**
     * @brief CalcEndPointSeg : 根据线段端点和中间点，计算另一个端点坐标
     * @param tStartPt
     * @param tMidPt
     * @return
     */
    static TPointd CalcEndPointSeg(TPointd tStartPt, TPointd tMidPt);

    /**
     * @brief Point2LineDis : 计算点到直线的距离
     * @param tPt
     * @param tLineStartPt
     * @param tLineEndPt
     * @param tIntersectionPt : 输出交点
     * @return 返回距离
     */
    static double Point2LineDis(TPointd tPt, TPointd tLineStartPt, TPointd tLineEndPt, TPointd &tIntersectionPt);

    /**
     * @brief CalcPointToLineDisAndInterSec:计算点到直线的距离以及交点坐标
     * @param tPt
     * @param tLineStartPt
     * @param tLineEndPt
     * @param tIntersectionPt: 输出交点
     * @return 返回距离
     */
    static double CalcPointToLineDisAndInterSec(TPointd tPt, TPointd tLineStartPt, TPointd tLineEndPt, TPointd &tIntersectionPt);

    ///
    /// \brief 已知直角三角形的直角点和一个边点、另一个边的长度，求另一个边的点坐标
    /// \param tPtRightAngle ： 直角点
    /// \param tPtSide ： 边点
    /// \param ilength ： 另一个边长度
    /// \param tPt1 ：另一个边点1
    /// \param tPt2 ：另一个边点2，和点1是对称的
    ///
    static bool CalcPointByRightTriangleTwoPtAndOneSideLength(TPointd tPtRightAngle, TPointd tPtSide, int ilength, TPointd &tPt1, TPointd &tPt2);

    ///
    /// \brief 已知两个点和所连线段的中点，求线段中点处指定长度处的两个点
    /// \param tStart ：线段起点
    /// \param tEnd ： 线段终点
    /// \param ilen ： 指定长度
    /// \param tPtToStart ：返回离起始点近的那个点
    /// \param tPtToEnd：返回离终止点近的那个点
    ///
    static void CalcPointByThreePointAndLength(TPointd tStart, TPointd tEnd, int ilen, TPointd &tPtToStart, TPointd &tPtToEnd, TPointd &tPtMid);
};

#endif // CGEOMETRYALGORITHM_H
