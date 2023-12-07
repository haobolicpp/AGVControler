/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2023-04-20 10:58:56
 * @LastEditors: yang.cheng yang.cheng@bgirobot.com
 * @LastEditTime: 2023-06-15 15:38:45
 * @FilePath: /agv_controller/src/agv_rvd_ctrl/AgvRvdTrans.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include "AgvRvdCtrl.h"

using namespace std;
using namespace Eigen;

/**
 * @name: ConvertToIsomerty
 * @des:  转换R33 T31一维数组至EIGEN-SE3描述
 * @param {double} *R33
 * @param {double} *T31
 * @param {Isometry3d} &m_Trans
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int ConvertToIsomerty(const double *R33, const double *T31, Isometry3d &m_Trans)
{
    int i, j;
    Matrix3d m_R33;  //旋转矩阵
    Vector3d m_T31;

    if (R33 == NULL || T31 == NULL)
    {
        return -1;
    }

    for (i = 0; i < 3; i++)
    {
        for(j = 0; j < 3; j++)
        {
            m_R33(i, j) = R33[i * 3 + j];
        }
    }

    m_T31(0) = T31[0];
    m_T31(1) = T31[1];
    m_T31(2) = T31[2];

    m_Trans.rotate(m_R33);
    m_Trans.pretranslate(m_T31);

    return 0;
}


/**
 * @name: AgvRvdGetRPYFromMR
 * @des:  根据旋转矩阵计算RPY
 * @param {double} *R33
 * @param {double} &roll
 * @param {double} &pitch
 * @param {double} &yaw
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int AgvRvdCtrl::CalcTransActuatorToGoal(const double *R33CameraToTag, 
                            const double *T31CameraToTag)
{
    int s32Ret;

    Isometry3d m_TransCameraToTag = Isometry3d::Identity();
    Isometry3d m_TransRobotToCamera = Isometry3d::Identity();
    Isometry3d m_TransRobotToTool = Isometry3d::Identity();
    Isometry3d m_TransTagToGoalA = Isometry3d::Identity();
    Isometry3d m_TransTagToGoalB = Isometry3d::Identity();

    Isometry3d m_TransRobotToGoalA = Isometry3d::Identity();
    Isometry3d m_TransRobotToGoalB = Isometry3d::Identity();

    //变换数据结构至Eigen-Transform
    ConvertToIsomerty(R33CameraToTag, T31CameraToTag, m_TransCameraToTag);
    std::cout << "m_TransCameraToTag:---------" << endl;
    std::cout << "Rotation:----" << endl;
    std::cout << (m_TransCameraToTag.rotation()) << endl;
    std::cout << "Tranlation:----" << endl;
    std::cout << (m_TransCameraToTag.translation()) << endl;

//发送至上位机 数据用于显示和标定-------------------------------------------
    for(int i = 0; i < 4; i++)
    {
        for(int j = 0; j < 4; j++)
        {
            f32TransCameraToTag_[4*i + j] = m_TransCameraToTag(i, j);
        }
    }
    Vector3d eulerAngle = m_TransCameraToTag.rotation().eulerAngles(2,1,0);
    f32TransCameraToTag_[16] = (float)eulerAngle(0);
    f32TransCameraToTag_[17] = (float)eulerAngle(1);
    f32TransCameraToTag_[18] = (float)eulerAngle(2);
    f32TransCameraToTag_[19] = 0.0;

    // std::cout << "RPY:----" << endl;
    // std::cout << f32TransCameraToTag_[16] << endl;
    // std::cout << f32TransCameraToTag_[17] << endl;
    // std::cout << f32TransCameraToTag_[18] << endl;


    // Eigen::AngleAxisd rollAngle(AngleAxisd(eulerAngle(2),Vector3d::UnitX()));
    // Eigen::AngleAxisd pitchAngle(AngleAxisd(eulerAngle(1),Vector3d::UnitY()));
    // Eigen::AngleAxisd yawAngle(AngleAxisd(eulerAngle(0),Vector3d::UnitZ()));
    
    // Eigen::Matrix3d rotation_matrix;
    // rotation_matrix=yawAngle*pitchAngle*rollAngle;

    // std::cout << "Rotation matrix:\n" << rotation_matrix << std::endl;
    // return 0;


    ConvertToIsomerty(R33RobotToCamera, T31RobotToCamera, m_TransRobotToCamera);
    std::cout << "m_TransRobotToCamera:---------" << endl;
    std::cout << "Rotation:----" << endl;
    std::cout << (m_TransRobotToCamera.rotation()) << endl;
    std::cout << "Tranlation:----" << endl;
    std::cout << (m_TransRobotToCamera.translation()) << endl;
    ConvertToIsomerty(R33RobotToTool, T31RobotToTool, m_TransRobotToTool);
    std::cout << "m_TransRobotToTool:---------" << endl;
    std::cout << "Rotation:----" << endl;
    std::cout << (m_TransRobotToTool.rotation()) << endl;
    std::cout << "Tranlation:----" << endl;
    std::cout << (m_TransRobotToTool.translation()) << endl;

    ConvertToIsomerty(R33TagToGoalA, T31TagToGoalA, m_TransTagToGoalA);
    std::cout << "m_TransTagToGoalA:---------" << endl;
    std::cout << "Rotation:----" << endl;
    std::cout << (m_TransTagToGoalA.rotation()) << endl;
    std::cout << "Tranlation:----" << endl;
    std::cout << (m_TransTagToGoalA.translation()) << endl;

    ConvertToIsomerty(R33TagToGoalB, T31TagToGoalB, m_TransTagToGoalB);
    std::cout << "m_TransTagToGoalB:---------" << endl;
    std::cout << "Rotation:----" << endl;
    std::cout << (m_TransTagToGoalB.rotation()) << endl;
    std::cout << "Tranlation:----" << endl;
    std::cout << (m_TransTagToGoalB.translation()) << endl;
    
    
    m_TransRobotToGoalA = m_TransRobotToCamera * m_TransCameraToTag * m_TransTagToGoalA;
    std::cout << "m_TransRobotToGoalA:---------" << endl;
    std::cout << "Rotation:----" << endl;
    std::cout << (m_TransRobotToGoalA.rotation()) << endl;
    std::cout << "Tranlation:----" << endl;
    std::cout << (m_TransRobotToGoalA.translation()) << endl;

    m_TransRobotToGoalB = m_TransRobotToCamera * m_TransCameraToTag * m_TransTagToGoalB;
    std::cout << "m_TransRobotToGoalB:---------" << endl;
    std::cout << "Rotation:----" << endl;
    std::cout << (m_TransRobotToGoalB.rotation()) << endl;
    std::cout << "Tranlation:----" << endl;
    std::cout << (m_TransRobotToGoalB.translation()) << endl;

    //-----------------------判断目标是否能被机器人校正，并赋值机器人执行位置-------------------------------------
    double theta;
    double axisZ;
    Vector3d v3dAxis;

    AngleAxisd m_RvRobotToGoalA = m_RvRobotToGoalA.fromRotationMatrix(m_TransRobotToGoalA.rotation());
    theta = m_RvRobotToGoalA.angle();
    v3dAxis = m_RvRobotToGoalA.axis();
    axisZ = v3dAxis(2);
    std::cout << "R: " << endl << m_RvRobotToGoalA.angle() << endl;
    std::cout << "V: " << endl << m_RvRobotToGoalA.axis() << endl;
    validGoalA = false;
    if (fabs(theta) < (3.14/180.0)) //误差允许1°  换算为轴角模式后 轴偏差小于1° 认为轴重合 类似于死区判定
    {
        thetaRobotGoalA = 0; //theta
        validGoalA = true;
    }
    else //存在旋转 
    {
        if (fabs(axisZ) < 0.95) //只能校正Z方向旋转
        {
            validGoalA = false;
        }

        if (axisZ * theta > 1e-6) //正向
        {
            validGoalA = true;
            thetaRobotGoalA = theta;
        }
        else if (axisZ * theta < -1e-6) //反向
        {
            validGoalA = true;
            thetaRobotGoalA = -theta;
        }
        else
        {
            validGoalA = false;
        }
    }
    //有效则赋值空间位置
    if (validGoalA == true)
    {
        T31RobotToGoalA[0] = m_TransRobotToGoalA.translation()(0); //x
        T31RobotToGoalA[1] = m_TransRobotToGoalA.translation()(1); //y
        T31RobotToGoalA[2] = m_TransRobotToGoalA.translation()(2); //z
        std::cout << "GoalA:" << T31RobotToGoalA[0] << T31RobotToGoalA[1] << T31RobotToGoalA[2] << endl;
    }
    else
    {
        std::cout << "Failed GoalA" << endl;
        return -1;
    }


    AngleAxisd m_RvRobotToGoalB = m_RvRobotToGoalB.fromRotationMatrix(m_TransRobotToGoalA.rotation());
    theta = m_RvRobotToGoalB.angle();
    v3dAxis = m_RvRobotToGoalB.axis();
    axisZ = v3dAxis(2);
    std::cout << "R: " << endl << m_RvRobotToGoalB.angle() << endl;
    std::cout << "V: " << endl << m_RvRobotToGoalB.axis() << endl;
    validGoalB = false;
    if (fabs(theta) < (3.14/180.0)) //误差允许1°
    {
        thetaRobotGoalB = 0; //theta
        validGoalB = true;
    }
    else //存在旋转 
    {
        if (fabs(axisZ) < 0.95) //只能校正Z方向旋转
        {
            validGoalB = false;
        }

        if (axisZ * theta > 1e-6) //正向
        {
            validGoalB = true;
            thetaRobotGoalB = theta;
        }
        else if (axisZ * theta < -1e-6) //反向
        {
            validGoalB = true;
            thetaRobotGoalB = -theta;
        }
        else
        {
            validGoalB = false;
        }
    }
    //有效则赋值空间位置
    if (validGoalB == true)
    {
        T31RobotToGoalB[0] = m_TransRobotToGoalB.translation()(0); //x
        T31RobotToGoalB[1] = m_TransRobotToGoalB.translation()(1); //y
        T31RobotToGoalB[2] = m_TransRobotToGoalB.translation()(2); //z
        std::cout << "GoalB:" << T31RobotToGoalB[0] << T31RobotToGoalB[1] << T31RobotToGoalB[2] << endl;
    }
    else
    {
        std::cout << "Failed GoalB" << endl;
        return -1;
    }


    return 0;
    

    // int i, j;

    // Matrix3d m_R33CameraToTag;  //旋转矩阵
    // AngleAxisd m_RvCameraToTag; //轴角

    // for (i = 0; i < 3; i++)
    // {
    //     for(j = 0; j < 3; j++)
    //     {
    //         m_R33CameraToTag(i, j) = R33CameraToTag[i * 3 + j];
    //     }
    // }
    // std::cout << "m_R33CameraToTag:" << endl;
    // std::cout << m_R33CameraToTag << endl;

    // m_RvCameraToTag = m_RvCameraToTag.fromRotationMatrix(m_R33CameraToTag);
    // std::cout << "m_RvCameraToTag:" << endl;
    // std::cout << "R: " << endl << m_RvCameraToTag.angle() << endl;
    // std::cout << "V: " << endl << m_RvCameraToTag.axis() << endl;

    // //Transform Camera->Tag 构建
    // Isometry3d m_TransCameraToTag = Isometry3d::Identity();
    // m_TransCameraToTag.rotate(m_RvCameraToTag);
    // m_TransCameraToTag.pretranslate(Eigen::Vector3d(T31CameraToTag[0], T31CameraToTag[1], T31CameraToTag[2]));
    // std::cout << (m_TransCameraToTag.rotation()) << endl;
    // std::cout << (m_TransCameraToTag.translation()) << endl;

}



