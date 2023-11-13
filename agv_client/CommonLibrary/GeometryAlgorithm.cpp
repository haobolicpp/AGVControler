#include <math.h>
#include <QtMath>
#include "GeometryAlgorithm.h"

CGeometryAlgorithm::CGeometryAlgorithm()
{

}

double CGeometryAlgorithm::DegToRad(double deg)
{
    return deg * (PI / 180);
}

double CGeometryAlgorithm::RadToDeg(double rad)
{
    return 180 * rad / PI;
}

double CGeometryAlgorithm::Point2PointAngle(double dx1, double dy1, double dx2, double dy2)
{
    double deltaX = dx2 - dx1;
    double deltaY = dy2 - dy1;
    double dis = std::sqrt(deltaX*deltaX + deltaY*deltaY);
    double dret = 0.0;

    if (DOUBLE_IS_ZERO(dis))
    {
        return 0.0;
    }

    dret = std::acos(deltaX / dis); //acos返回0~PI
    dret = RadToDeg(dret);

    if (deltaY < 0)
    {
        dret = -dret;
    }

    return dret;
}

TPointd CGeometryAlgorithm::CalcEndPointSeg(TPointd tStartPt, TPointd tMidPt)
{
    TPointd tEndPt;
    tEndPt.dX = 2 * tMidPt.dX - tStartPt.dX;
    tEndPt.dY = 2 * tMidPt.dY - tStartPt.dY;
    return tEndPt;
}

double CGeometryAlgorithm::Point2LineDis(TPointd tPt, TPointd tLineStartPt, TPointd tLineEndPt, TPointd &tIntersectionPt)
{
    //向量a的坐标表示
    TPointd A;
    A.dX = tPt.dX - tLineStartPt.dX;
    A.dY = tPt.dY - tLineStartPt.dY;
    //向量b的坐标表示
    TPointd B;
    B.dX = tLineEndPt.dX - tLineStartPt.dX;
    B.dY = tLineEndPt.dY - tLineStartPt.dY;
    if(0 == B.dX && 0 == B.dY)
    {
        assert("error");
    }
    //有公式可得向量c的模长|c| = a.b/|b|
    //a.b = x1x2 + y1y2
    //向量c的坐标表示  c = tIntersectionPt - tLineStartPt = ((a.b)/|b|^2).b
    TPointd C;
    C.dX = ((A.dX * B.dX + A.dY * B.dY ) / (double)(B.dX*B.dX + B.dY * B.dY)) * B.dX;
    C.dY = ((A.dX * B.dX + A.dY * B.dY ) / (double)(B.dX*B.dX + B.dY * B.dY)) * B.dY;
    tIntersectionPt.dX = C.dX + tLineStartPt.dX ;
    tIntersectionPt.dY = C.dY + tLineStartPt.dY;
    //由向量 e = a - c 再取e的模即可得到点到直线的距离
    double dDistance = sqrt((A.dX -C.dX) * (A.dX -C.dX) + (A.dY -C.dY) * (A.dY -C.dY));
    return dDistance;
}

double CGeometryAlgorithm::CalcPointToLineDisAndInterSec(TPointd tPt, TPointd tLineStartPt, TPointd tLineEndPt, TPointd &tIntersectionPt)
{
    double a = (tLineStartPt.dY - tLineEndPt.dY) / (double)(tLineStartPt.dX - tLineEndPt.dX);
    double b = (tLineStartPt.dY - a * tLineStartPt.dY);
    double m = tPt.dX + a * tPt.dY;
    tIntersectionPt.dX = (m-a*b)/(a*a+1);
    tIntersectionPt.dY = a*((m-a*b)/(a*a+1)) + b;
    double dDistance = qAbs(a*tPt.dX + b - tPt.dY)/sqrt(a*a+1);
    return dDistance;
}

bool CGeometryAlgorithm::CalcPointByRightTriangleTwoPtAndOneSideLength(TPointd tPtRightAngle, TPointd tPtSide, int ilength, TPointd &tPt1, TPointd &tPt2)
{
    //参考https://zhidao.baidu.com/question/345319724.html
    //设直角点A，一个端点已知B，求C
    //公式1：直角两个向量相乘为0 (AB*AC = 0，即(Xc-Xa)(Xb-Xa)+(Yc-Ya)(Yb-Ya) = 0)
    //公式2：指定的长度ilength用坐标去算(ilength^2=(Xc-Xa)^2+(Yc-Ya)^2)
    //关键是不要全部展开，求出1中的Yc-Ya，代入2即可解出
    //结果Xc = Xa ± l(Ya-Yb)/sqrt((Yb-Ya)^2+(Xb-Xa)^2)
    //Yc = Ya ± l(Xb-Xa)/sqrt((Yb-Ya)^2+(Xb-Xa)^2)


    double dDis = sqrt((tPtSide.dY-tPtRightAngle.dY)*(tPtSide.dY-tPtRightAngle.dY) + (tPtSide.dX-tPtRightAngle.dX)*(tPtSide.dX-tPtRightAngle.dX));
    if (dDis == 0)
    {
        return false;
    }

    tPt1.dX = tPtRightAngle.dX + ilength*(tPtRightAngle.dY-tPtSide.dY)/dDis;
    tPt1.dY = tPtRightAngle.dY + ilength*(tPtSide.dX-tPtRightAngle.dX)/dDis;

    tPt2.dX = tPtRightAngle.dX - ilength*(tPtRightAngle.dY-tPtSide.dY)/dDis;
    tPt2.dY = tPtRightAngle.dY - ilength*(tPtSide.dX-tPtRightAngle.dX)/dDis;

    return true;
}

void CGeometryAlgorithm::CalcPointByThreePointAndLength(TPointd tStart, TPointd tEnd, int ilen, TPointd &tPtToStart, TPointd &tPtToEnd, TPointd &tPtMid)
{
    //原理：通过中点进行坐标偏移
    tPtMid.dX = (tStart.dX+tEnd.dX)/2;
    tPtMid.dY = (tStart.dY+tEnd.dY)/2;

    double dth = atan(fabs(tEnd.dY-tStart.dY)/fabs(tEnd.dX-tStart.dX));

    if ((tStart.dX<tEnd.dX) && (tStart.dY>tEnd.dY))
    {
        tPtToStart.dX = tPtMid.dX - ilen*cos(dth);
        tPtToStart.dY = tPtMid.dY + ilen*sin(dth);

        tPtToEnd.dX = tPtMid.dX + ilen*cos(dth);
        tPtToEnd.dY = tPtMid.dY - ilen*sin(dth);
    }
    else if ((tStart.dX>tEnd.dX) && (tStart.dY>tEnd.dY))
    {
        tPtToStart.dX = tPtMid.dX + ilen*cos(dth);
        tPtToStart.dY = tPtMid.dY + ilen*sin(dth);

        tPtToEnd.dX = tPtMid.dX - ilen*cos(dth);
        tPtToEnd.dY = tPtMid.dY - ilen*sin(dth);
    }
    else if ((tStart.dX<tEnd.dX) && (tStart.dY<tEnd.dY))
    {
        tPtToStart.dX = tPtMid.dX - ilen*cos(dth);
        tPtToStart.dY = tPtMid.dY - ilen*sin(dth);

        tPtToEnd.dX = tPtMid.dX + ilen*cos(dth);
        tPtToEnd.dY = tPtMid.dY + ilen*sin(dth);
    }
    else
    {
        tPtToStart.dX = tPtMid.dX + ilen*cos(dth);
        tPtToStart.dY = tPtMid.dY - ilen*sin(dth);

        tPtToEnd.dX = tPtMid.dX - ilen*cos(dth);
        tPtToEnd.dY = tPtMid.dY + ilen*sin(dth);
    }
}
