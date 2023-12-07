/*
 * @Author: yang.cheng yang.cheng@bgirobot.com
 * @Date: 2023-04-10 10:38:05
 * @LastEditors: yang.cheng yang.cheng@bgirobot.com
 * @LastEditTime: 2023-09-12 17:51:17
 * @FilePath: /AGVController-Server-V1.0/include/agv_rvd_ctrl/agv_rvd_ctrl.h
 * @Description: AGV二次定位与对接实现 相机取图->靶标位姿估计->调整执行机构
 */

#include <pthread.h>
#include <mqueue.h>

#include "agv_type.h"
#include "agv_cmd.h"
#include "AgvCtrl.h"
#include "AgvTf.h"
#include "AgvRvdCtrl.h"
#include "MvCameraControl.h"

#include "apriltag.h"
#include "apriltag_pose.h" //pose estimation lib
#include "tag36h11.h"
#include "common/image_u8.h"
#include "common/pjpeg.h"
#include "common/zarray.h"

#include "hitbot_interface.h"
#include "ControlBeanEx.h"
/**
 * @brief AgvRvdCtrl构造
 * @param 
 * @return
 */
AgvRvdCtrl::AgvRvdCtrl(AgvCtrl *pt_agv_ctrl, AgvTf *poAgvTf)
{
    sprintf(pImageName_, "/home/%s/tag_record/tag_img_default.jpeg", getlogin());
    sprintf(pImageDir_, "/home/%s/tag_record", getlogin());
    
    bHasInited =  false;
    bHasStarted = false;
    s32CtrlFreq = 10;
}

/**
 * @brief AgvRvdCtrl析构
 * @param 
 * @return
 */
AgvRvdCtrl::~AgvRvdCtrl()
{
    bHasStarted = false;
    CameraDeInit();
}

/**
 * @brief AgvRvdCtrl初始化
 * @param 
 * @return
 */
int AgvRvdCtrl::Init()
{
    int s32Ret;

    if (s32CtrlFreq <= 1)
    {
        printf("AgvRvdCtrl too small control freqency!");
        return -1;
    }

    //创建一个默认目录用以保存历史TAG
    struct stat st = {0};
    if (stat(pImageDir_, &st) == -1) 
    {
        int result = mkdir(pImageDir_, 0777);
        if(result != 0) 
        {
            printf("AgvRvdCtrl Failed to create tag directory.\n");
            return -1;
        }
    }

    State = AgvRvdState::INIT;
    //静态坐标
    s32Ret = StaticTransInit();
    if (s32Ret < 0)
    {
        printf("StaticTrans Init Faild!\n");
        return -1;
    }
    //相机系统初始化
    s32Ret = CameraInit();
    if (s32Ret < 0)
    {
        printf("AgvRvdCtrl Camera Init Faild!\n");
        return -1;
    }
    //靶标监测系统初始化
    s32Ret = TargetDetectInit();
    if (s32Ret < 0)
    {
        printf("AgvRvdCtrl Target Detect Init Faild!\n");
        return -1; 
    }
    //执行机构初始化TODO

#if 1
    s32Ret = ActuatorInit();
    if (s32Ret < 0)
    {
        printf("AgvRvdCtrl Actuator Init Faild!\n");
        return -1; 
    }

    //Test ---- 尝试获取一张图像

    s32Ret = CameraGetImage();
    if (s32Ret < 0)
    {
        printf("AgvRvdCtrl Get Img Faild!\n");
        return -1;
    }
    sleep(1);
    //根据TAG计算机器人坐标系下目标的位姿
    s32Ret = TargetFrameCalc();
    if (s32Ret < 0)
    {
        printf("AgvRvdCtrl Goal Invalid\n");
        return -1;
    }
    //执行机构运行至目标
    printf("ActuatorExecute..\n");
    s32Ret = ActuatorExecute();
    if (s32Ret < 0)
    {
        printf("AgvRvdCtrl Actuator Execute Failed!\n");
        return -1;
    }
    //执行机构复位至存放点
    sleep(5);
    printf("ActuatorHome..\n");
    s32Ret = ActuatorHome();
    if (s32Ret < 0)
    {
        printf("AgvRvdCtrl Actuator Home Failed!\n");
        return -1;
    }
#endif


    //主框架要求线程创建消息队列
    struct mq_attr t_mq_attr = {0};
	t_mq_attr.mq_flags = 0; //Block Mode
	t_mq_attr.mq_msgsize = sizeof(st_robot_msg);
	t_mq_attr.mq_maxmsg = 8;
    mq_unlink("/q_agv_rvd");
    mode_t omask;
    omask = umask(0); //返回之前的文件权限默认值，同时修改默认的umask数值为0，这样后面设置文件权限的时候，umask值无影响了（创建文件时，文件权限的计算方式是：umask取反 & 要设定的文件权限）
    this->qRvdCtrl = mq_open("/q_agv_rvd", O_RDWR | O_CREAT | O_NONBLOCK , (S_IRWXU | S_IRWXG | S_IRWXO) /* 777 其他用户可修改*/,  &t_mq_attr);
    umask(omask);
    if (this->qRvdCtrl < 0)
    {
        printf("qRvdCtrl create failed with error:%s\n",strerror(errno));
        return -1;
    }
    printf("AgvRvdCtrl Init!\n");

    //完成初始化并设置状态
    State = AgvRvdState::INIT;
    bHasInited = true;

    return 0;
}



/**
 * @brief AgvRvdCtrl工作线程启动
 * @param 
 * @return
 */
int AgvRvdCtrl::Start()
{
    int s32Ret;

    if(!bHasInited || bHasStarted)
    {
        printf("AgvRvdCtrl Start failed!\n");
        return -1;
    }

    //状态改变放在线程创建之前 防止线程创建后直接退出
    bHasStarted = true;
    pthread_attr_t pthread_attr;
    struct sched_param sched_param;
    pthread_attr_init(&pthread_attr);
    pthread_attr_setinheritsched(&pthread_attr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&pthread_attr, SCHED_OTHER);
    sched_param.sched_priority = 10;
    pthread_attr_setschedparam(&pthread_attr, &sched_param);

    s32Ret = pthread_create(&this->thRvdCtrl, &pthread_attr, LoopRvdCtrlStatic, (void *)(this));
    assert(s32Ret >= 0);

    printf("AgvRvdCtrl Run!\n");
    return 0;
}


/**
 * @brief AgvRvdCtrl工作线程入口
 * @param arg 当前类指针 
 * @return void
 */
void *AgvRvdCtrl::LoopRvdCtrlStatic(void *arg)
{
    AgvRvdCtrl *poRvdCtrl = (AgvRvdCtrl *)arg;
    poRvdCtrl->LoopRvdCtrl();
    return NULL;
}
/**
 * @brief AgvPlanner工作线程实体
 * @return 
 */
int AgvRvdCtrl::LoopRvdCtrl()
{
    int s32Ret;
    struct timeval t_tmptv;
    fd_set tFdReadSet;
    TRobotMsg tAgvMsg;

    int s32FdSetMax = qRvdCtrl + 1;
    int usFromCtrlFreq = floor(1000000.0/s32CtrlFreq);
    State = AgvRvdState::READY;
	t_tmptv.tv_sec = 0;
    t_tmptv.tv_usec = 200000; 

    while(bHasStarted)
    {
        FD_ZERO(&tFdReadSet);
        FD_SET(qRvdCtrl, &tFdReadSet);
        s32Ret = select(s32FdSetMax, &tFdReadSet, NULL, NULL, &t_tmptv);
        if (s32Ret < 0)
        {
            printf("AgvRvdCtrl Q Loop Error %s\n", strerror(errno));
            sleep(1);
            continue;
        }
        //控制周期处理------------------------------------------------
        else if (s32Ret == 0)
        {
            t_tmptv.tv_sec = 0;
            t_tmptv.tv_usec = 500000; 
            //printf("agv_rvd_ctrl loop..\n");
            continue;
        }

        //异步消息处理------------------------------------------------
        if (FD_ISSET(qRvdCtrl, &tFdReadSet))
        {
            memset(&tAgvMsg, 0, sizeof(st_robot_msg));
            s32Ret = mq_receive(qRvdCtrl, (char *)&tAgvMsg,
                        sizeof(st_robot_msg), NULL);
            if (s32Ret < 0)
            {
                printf("ERROR: AgvRvdCtrl Recv Msg Failed %d\n", errno);
                continue;
            }
            AsyncMsgHanler(&tAgvMsg);
            RobotMsgRelease(&tAgvMsg);
            // printf("AgvRvdCtrl Q Get Msg with C[%2d]-T[%2d]\n", 
            //     tAgvMsg.t_msg_header.u16_class, tAgvMsg.t_msg_header.u16_type);
        }

    }
    return 0;
}



/**
 * @brief 模块功能请求 一般被其他模块调用
 * @param 
 * @return 0 成功 -1失败
 */
int AgvRvdCtrl::AsyncMsgPost(TRobotMsg *ptAgvMsg)
{
    int ret;
    ret = mq_send(qRvdCtrl, (char *)ptAgvMsg, sizeof(st_robot_msg), 0);
    if (ret < 0)
    {
        RobotMsgRelease(ptAgvMsg);
        return -1;
    }
    return 0;
}

/**
 * @name: AsyncMsgHanler
 * @des:  异步消息响应
 * @param {st_robot_msg} *ptAgvMsg
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int AgvRvdCtrl::AsyncMsgHanler(TRobotMsg *ptAgvMsg)
{
    int s32Ret = 0;

    switch (State)
    {
    case AgvRvdState::READY:
        s32Ret = AsyncMsgHandlerInReady(ptAgvMsg);
        break;

    default: //非法状态与消息处理
        s32Ret = -1;
        break;
    }
    return s32Ret;
}

/**
 * @name: 二次定位对接模块 READY状态下的消息处理
 * @des: 
 * @param {st_robot_msg} *ptAgvMsg
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int AgvRvdCtrl::AsyncMsgHandlerInReady(TRobotMsg *ptAgvMsg)
{
    int s32Ret = 0;

    if (ptAgvMsg->t_msg_header.u16_class == AGV_CMD_C_FILE 
        && ptAgvMsg->t_msg_header.u16_type == AGV_CMD_T_RVD_IMAGE_REQ)
    {
        s32Ret = CameraGetImage();
        if (s32Ret < 0)
        {
            printf("AgvRvdCtrl Get Img Faild!\n");
            return -1;
        }
//构造Tag图片消息返回至Client端----------------------------
        //1读取最近保存的TAG图片
        FILE* fp = fopen(pImageName_, "r");
        if (fp == NULL) {
            printf("Failed to open file.\n");
            return -1;
        }
        fseek(fp, 0, SEEK_END);
        long size = ftell(fp);
        rewind(fp);
        char* buffer = (char*)malloc(size + 1);
        fread(buffer, size, 1, fp);
        buffer[size] = '\0';
        fclose(fp);
        
        if ((size + 1) >= 16 * 1024 * 1024)
        {
            printf("Tag Image Size Limited.\n");
            return -1;
        }

        //1构造消息并
        TRobotMsg tAgvMsg = {0};
        TRobotMsgHeader tAgvMsgHeader = {0};
        tAgvMsgHeader.u16_src = 0x10;
        tAgvMsgHeader.u16_dest = 1;
        tAgvMsgHeader.u16_class = AGV_CMD_C_FILE;
        tAgvMsgHeader.u16_type = AGV_CMD_T_RVD_IMAGE_RESP;
        tAgvMsgHeader.u32_sq = 0;
        tAgvMsgHeader.s32_len = size + 1;
        RobotMsgInit(&tAgvMsg, tAgvMsgHeader, (uint8_t *)buffer);
        free(buffer); //构造消息完毕则释放Buffer

        //3投递至通信线程以发送至Client端
        int s32QcmdFrom = ptAgvMsg->t_msg_header.u32_sq;
        s32Ret = mq_send(s32QcmdFrom, (char *)&tAgvMsg, sizeof(st_robot_msg), 0);
        if(s32Ret < 0)
        {
            printf("ERROR: Rvd Tcp Post %d\n", errno);
            return -1;
        }
        //sleep(3);
        //根据TAG计算机器人坐标系下目标的位姿
        s32Ret = TargetFrameCalc();
        // if (s32Ret < 0)
        // {
        //     printf("AgvRvdCtrl Goal Invalid\n");
        //     return -1;
        // }

        st_robot_msg tAgvMsg2;
        tAgvMsgHeader.u16_src = 0x10;
        tAgvMsgHeader.u16_dest = 1;
        tAgvMsgHeader.u16_class = AGV_CMD_C_FILE;
        tAgvMsgHeader.u16_type = AGV_CMD_T_RVD_POSE_RESP;
        tAgvMsgHeader.u32_sq = 0;
        tAgvMsgHeader.s32_len = 20 * 4;
        RobotMsgInit(&tAgvMsg2, tAgvMsgHeader, (uint8_t *)f32TransCameraToTag_);
        s32Ret = mq_send(s32QcmdFrom, (char *)&tAgvMsg2, sizeof(st_robot_msg), 0);
        if(s32Ret < 0)
        {
            printf("ERROR: Rvd Tcp Post %d\n", errno);
            return -1;
        }

    }

    return s32Ret;
}



//------------------------------------------------------------------------------------------
/**
 * @name: CameraInit
 * @des: 通过GIGE对相机进行初始化
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int AgvRvdCtrl::CameraInit()
{
    int s32Ret;

    //清空GIGE设备列表
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    //枚举设备
    s32Ret = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    if (MV_OK != s32Ret)
    {
        printf("MV_CC_EnumDevices fail! nRet [%x]\n", s32Ret);
        return -1;
    }
    if (stDeviceList.nDeviceNum > 0)
    {
        for (int i = 0; i < stDeviceList.nDeviceNum; i++)
        {
            printf("[device %d]:\n", i);
            MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[i];
            if (NULL == pDeviceInfo)
            {
                break;
            } 
            CameraInfo(pDeviceInfo);
        }  
    } 
    else
    {
        printf("Find No Devices!\n");
        return -1;
    }

    //创建设备句柄 当前仅有相机设备0
    s32Ret = MV_CC_CreateHandle(&pCameraHandle, stDeviceList.pDeviceInfo[0]);
    if (MV_OK != s32Ret)
    {
        printf("MV_CC_CreateHandle fail! nRet [%x]\n", s32Ret);
        return -1;
    }

    //打开设备
    s32Ret = MV_CC_OpenDevice(pCameraHandle);
    if (MV_OK != s32Ret)
    {
        printf("MV_CC_OpenDevice fail! nRet [%x]\n", s32Ret);
        return -1;
    }

    //  ch: 探测网络最佳包大小(只对GigE相机有效)
    if (stDeviceList.pDeviceInfo[0]->nTLayerType == MV_GIGE_DEVICE)
    {
        int nPacketSize = MV_CC_GetOptimalPacketSize(pCameraHandle);
        if (nPacketSize > 0)
        {
            s32Ret = MV_CC_SetIntValue(pCameraHandle,"GevSCPSPacketSize",nPacketSize);
            if(s32Ret != MV_OK)
            {
                printf("Warning: Set Packet Size fail nRet [0x%x]!\n", s32Ret);
            }
        }
        else
        {
            printf("Warning: Get Packet Size fail nRet [0x%x]!\n", nPacketSize);
        }
    }

    //设置触发模式
    s32Ret = MV_CC_SetEnumValue(pCameraHandle, "TriggerMode", 0);
    if (MV_OK != s32Ret)
    {
        printf("MV_CC_SetTriggerMode fail! nRet [%x]\n", s32Ret);
        return -1;
    }

    // ch:获取数据包大小 | en:Get payload size
    memset(&stImageParam, 0, sizeof(MVCC_INTVALUE));
    s32Ret = MV_CC_GetIntValue(pCameraHandle, "PayloadSize", &stImageParam);
    if (MV_OK != s32Ret)
    {
        printf("Get PayloadSize fail! nRet [0x%x]\n", s32Ret);
        return -1;
    }

    return 0;
}


/**
 * @name: CameraDeInit
 * @des: 通过GIGE对相机进行反初始化
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int AgvRvdCtrl::CameraDeInit()
{
    // destroy handle
    MV_CC_DestroyHandle(pCameraHandle);
    return 0;
}
/**
 * @name: CameraInfo
 * @des: 根据GIGE枚举解析相机设备基本信息
 * @param {MV_CC_DEVICE_INFO*} pstMVDevInfo
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int AgvRvdCtrl::CameraInfo(MV_CC_DEVICE_INFO* pstMVDevInfo)
{
    if (NULL == pstMVDevInfo)
    {
        printf("The Pointer of pstMVDevInfo is NULL!\n");
        return -1;
    }
    if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE)
    {
        int nIp1 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
        int nIp2 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
        int nIp3 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
        int nIp4 = (pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);

        // ch:打印当前相机ip和用户自定义名字 | en:print current ip and user defined name
        printf("Device Model Name: %s\n", pstMVDevInfo->SpecialInfo.stGigEInfo.chModelName);
        printf("CurrentIp: %d.%d.%d.%d\n" , nIp1, nIp2, nIp3, nIp4);
        printf("UserDefinedName: %s\n\n" , pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
    }
    else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE)
    {
        printf("Device Model Name: %s\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chModelName);
        printf("UserDefinedName: %s\n\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
    }
    else
    {
        printf("Not support.\n");
    }

    return 0;
}

/**
 * @name: CameraGetImage
 * @des: 获取一张完整的JPG图片
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int AgvRvdCtrl::CameraGetImage()
{
    int s32Ret;
    unsigned char *pData = NULL;    //指向为图像流分配的内存
    unsigned char *pDataForSaveImage = NULL; //指向为暂存图片分配的内存
    // 开始取流
    s32Ret = MV_CC_StartGrabbing(pCameraHandle);
    if (MV_OK != s32Ret)
    {
        printf("MV_CC_StartGrabbing fail! nRet [%x]\n", s32Ret);
        return -1;
    }

    MV_FRAME_OUT_INFO_EX stImageInfo = {0};
    memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
    pData = (unsigned char *)malloc(sizeof(unsigned char) * stImageParam.nCurValue);
    if (NULL == pData)
    {
        return -1;
    }
    unsigned int nDataSize = stImageParam.nCurValue;

    //获取一帧完整的图片
    s32Ret = MV_CC_GetOneFrameTimeout(pCameraHandle, pData, nDataSize, &stImageInfo, 1000);
    if (MV_OK != s32Ret)
    {
        return -1;
    }
    printf("GetOneFrame, Width[%d], Height[%d], nFrameNum[%d]\n\n", 
                stImageInfo.nWidth, stImageInfo.nHeight, stImageInfo.nFrameNum);

    //为暂存图片分配内存
    pDataForSaveImage = (unsigned char*)malloc(stImageInfo.nWidth * stImageInfo.nHeight * 4 + 2048);
    if (NULL == pDataForSaveImage)
    {
        return -1;
    }
    // 填充存图参数
    MV_SAVE_IMAGE_PARAM_EX stSaveParam;
    memset(&stSaveParam, 0, sizeof(MV_SAVE_IMAGE_PARAM_EX));
    // 从上到下依次是：输出图片格式，输入数据的像素格式，提供的输出缓冲区大小，图像宽，
    // 图像高，输入数据缓存，输出图片缓存，JPG编码质量
    stSaveParam.enImageType = MV_Image_Jpeg; 
    stSaveParam.enPixelType = stImageInfo.enPixelType; 
    stSaveParam.nBufferSize = stImageInfo.nWidth * stImageInfo.nHeight * 4 + 2048;
    stSaveParam.nWidth      = stImageInfo.nWidth; 
    stSaveParam.nHeight     = stImageInfo.nHeight; 
    stSaveParam.pData       = pData;
    stSaveParam.nDataLen    = stImageInfo.nFrameLen;
    stSaveParam.pImageBuffer = pDataForSaveImage;
    stSaveParam.nJpgQuality = 80;

    s32Ret = MV_CC_SaveImageEx2(pCameraHandle, &stSaveParam);
    if(MV_OK != s32Ret)
    {
        printf("failed in MV_CC_SaveImage,nRet[%x]\n", s32Ret);
        return -1;
    }

    //保存图片
    time_t t = time(0);
    char timestr[256];
    strftime(timestr, 256, "%Y%m%d%H%M", localtime(&t));
    sprintf(pImageName_, "/home/%s/tag_record/tag_img_%s.jpeg", getlogin(), timestr);
    FILE* fp = fopen(pImageName_, "w");
    if (NULL == fp)
    {
        printf("fopen failed\n");
        return -1;
    }
    fwrite(pDataForSaveImage, 1, stSaveParam.nImageLen, fp);
    fclose(fp);
    printf("Save image succeed\n");

    // 停止取流
    s32Ret = MV_CC_StopGrabbing(pCameraHandle);
    if (MV_OK != s32Ret)
    {
        printf("MV_CC_StopGrabbing fail! nRet [%x]\n", s32Ret);
        return -1;
    }

    //释放内存
    free(pData);	
    pData = NULL;
    free(pDataForSaveImage);
    pDataForSaveImage = NULL;
    
    return 0;
}

/**
 * @name: TargetDetectInit
 * @des:  视觉靶标系统初始化
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int AgvRvdCtrl::TargetDetectInit()
{
    ptTagFamily = tag36h11_create();
    ptTagDetector = apriltag_detector_create();
    apriltag_detector_add_family(ptTagDetector, ptTagFamily);

    //Todo 相机内参 与 TAG尺寸初始化 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    //相机内参需要标定
    //tTagInfo.tagsize = 0.09;
    tTagInfo.tagsize = 0.0865;
    tTagInfo.det = NULL;
    tTagInfo.fx = 2.28471742e+03; 
    tTagInfo.fy = 2.28513822e+03;
    tTagInfo.cx = 1.28844375e+03;
    tTagInfo.cy = 1.00720163e+03;

    return 0;
}

/**
 * @name: TargetDetectDeInit
 * @des:  视觉靶标系统反初始化
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int AgvRvdCtrl::TargetDetectDeInit()
{
    apriltag_detector_destroy(ptTagDetector);
    tag36h11_destroy(ptTagFamily);

    return 0;
}


/**
 * @name: TargetFrameCalc
 * @des:  计算Tag在相机坐标系下的位姿
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int AgvRvdCtrl::TargetFrameCalc()
{
    int s32Ret = -1;
    image_u8_t *im = NULL;
    int err = 0;

    //1 读入Image
    // sprintf(pImageName, "/home/%s/Downloads/TagImage.jpeg", getlogin());
    pjpeg_t *pjpeg = pjpeg_create_from_file(pImageName_, 0, &err);
    if (pjpeg == NULL) {
        printf("pjpeg failed to load: %s, error %d\n", pImageName_, err);
        return -1;
    }
    im = pjpeg_to_u8_baseline(pjpeg);

    //2 解码Image获取每个Tag的信息
    zarray_t *detections = apriltag_detector_detect(ptTagDetector, im);

    //3 根据每个Tag信息以及相机内参 计算每个Tag相对于相机坐标系的位姿POSE
    apriltag_detection_t *det;
    apriltag_pose_t pose;
    double roll, pitch, yaw;
    for (int i = 0; i < zarray_size(detections); i++)
    {
        zarray_get(detections, i, &det);
        tTagInfo.det = det;
        estimate_tag_pose(&tTagInfo, &pose);
 
        cout << "x: " << pose.t ->data[0] << endl;
        cout << "y: " << pose.t ->data[1] << endl;
        cout << "z: " << pose.t ->data[2] << endl;

        s32Ret = CalcTransActuatorToGoal(pose.R->data, pose.t->data);

    }

    //TODO释放内存
    //estimate_tag_pose 是否带来内存泄漏问题
    apriltag_detections_destroy(detections);
    image_u8_destroy(im);
    pjpeg_destroy(pjpeg);

    return s32Ret;
}



//----------------------------------------------------------------------------------
/**
 * @name: ActuatorInit
 * @des: 初始化执行机构 后台建立服务线程 调试中断将影响执行机构的通信
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int AgvRvdCtrl::ActuatorInit()
{
    int s32Ret;

    net_port_initial();
    if (card_number_connect(45)) {
        ptHitbot = get_robot(45);
        s32Ret = ptHitbot->initial(1, 210);
        ptHitbot->unlock_position();
    }
    else
    {
        return -1;
    }

    //运行初始化测试
    s32Ret = ptHitbot->movej_angle(0, 0, 0, 0, 50.0, 0);
    printf("hitbot run [%d]\n", s32Ret);
    ptHitbot->wait_stop();
    s32Ret = ptHitbot->new_movej_xyz_lr(68.494, 18.824, -21.2046, -90.647, 50, 0, -1); //-1左手系
    printf("hitbot run [%d]\n", s32Ret);
    ptHitbot->wait_stop();

    return 0;
}

/**
 * @name: ActuatorExecute
 * @des:  根据计算得到的目标位姿GOALA-GOALB执行机构运行
 * 到达GOALA后直线运行到GOALB
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int AgvRvdCtrl::ActuatorExecute()
{
    int s32Ret;
    bool bRet;
    double x, y, z, theta;

    if (validGoalA == false || validGoalB == false)
    {
        return -1;
    }

    //GoalA m->mm  rad->deg
    x = T31RobotToGoalA[0] * 1000;
    y = T31RobotToGoalA[1] * 1000;
    z = T31RobotToGoalA[2] * 1000;
    theta = thetaRobotGoalA * 180.0 / 3.14159;

    bRet = ptHitbot->judge_in_range(x, y, z, theta);
    if (bRet == false)
    {
        printf("GoalA not in range of robot!\n");
        return -1;
    }
    s32Ret = ptHitbot->new_movej_xyz_lr(x, y, z, theta, 50, 0, -1);
    printf("hitbot runing to GoalA [%d]\n", s32Ret);
    ptHitbot->wait_stop();
    sleep(1);


    //GoalA m->mm  rad->deg
    x = T31RobotToGoalB[0] * 1000;
    y = T31RobotToGoalB[1] * 1000;
    z = T31RobotToGoalB[2] * 1000;
    theta = thetaRobotGoalB * 180.0 / 3.14159;
    bRet = ptHitbot->judge_in_range(x, y, z, theta);
    if (bRet == false)
    {
        printf("GoalB not in range of robot!\n");
        return -1;
    }

    s32Ret = ptHitbot->movel_xyz(x, y, z, theta, 50);
    printf("hitbot runing to GoalB [%d]\n", s32Ret);
    ptHitbot->wait_stop();
    sleep(1);

    return 0;
}

/**
 * @name: ActuatorHome
 * @des:  执行机构运行至复位点
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int AgvRvdCtrl::ActuatorHome()
{
    return 0;
}


/**
 * @name: 
 * @des: 
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int AgvRvdCtrl::StaticTransInit()
{
    // //机器人到相机的变换 9点标定
    // double R33RobotToCamera[9];
    // double T31RobotToCamera[3];
    // //机器人到工具坐标系的变换 机械尺寸给定
    // double R33RobotToTool[9];
    // double T31RobotToTool[3];
    // //靶标到目标的变换 机械给定
    // double R33TagToGoal[9];
    // double T31TagToGoal[3];

    R33RobotToCamera[0] = 0;
    R33RobotToCamera[1] = 0;
    R33RobotToCamera[2] = 1;
    R33RobotToCamera[3] = -1;
    R33RobotToCamera[4] = 0;
    R33RobotToCamera[5] = 0;
    R33RobotToCamera[6] = 0;
    R33RobotToCamera[7] = -1;
    R33RobotToCamera[8] = 0;
    T31RobotToCamera[0] = 0.10215;
    T31RobotToCamera[1] = -0.18;
    T31RobotToCamera[2] = -0.2855;

    R33RobotToTool[0] = 1;
    R33RobotToTool[1] = 0;
    R33RobotToTool[2] = 0;
    R33RobotToTool[3] = 0;
    R33RobotToTool[4] = 1;
    R33RobotToTool[5] = 0;
    R33RobotToTool[6] = 0;
    R33RobotToTool[7] = 0;
    R33RobotToTool[8] = 1;
    T31RobotToTool[0] = 0;
    T31RobotToTool[1] = 0;
    T31RobotToTool[2] = 0;

    R33TagToGoalA[0] = 0;
    R33TagToGoalA[1] = -1;
    R33TagToGoalA[2] = 0;
    R33TagToGoalA[3] = 0;
    R33TagToGoalA[4] = 0;
    R33TagToGoalA[5] = -1;
    R33TagToGoalA[6] = 1;
    R33TagToGoalA[7] = 0;
    R33TagToGoalA[8] = 0;
    T31TagToGoalA[0] = -0.15;
    T31TagToGoalA[1] = -0.15;
    T31TagToGoalA[2] = -0.15;

    R33TagToGoalB[0] = 0;
    R33TagToGoalB[1] = -1;
    R33TagToGoalB[2] = 0;
    R33TagToGoalB[3] = 0;
    R33TagToGoalB[4] = 0;
    R33TagToGoalB[5] = -1;
    R33TagToGoalB[6] = 1;
    R33TagToGoalB[7] = 0;
    R33TagToGoalB[8] = 0;
    T31TagToGoalB[0] = -0.15;
    T31TagToGoalB[1] = -0.15;
    T31TagToGoalB[2] = -0.10;


    return 0;
}