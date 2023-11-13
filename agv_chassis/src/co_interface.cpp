
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include <assert.h>
#include <mqueue.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "canfestival.h"
#include "global.hpp"
#include "co_interface.hpp"
#include "motor_controller.hpp"



mqd_t QCoIf = -1;
const char *QName = "/q_agv_co_svr";

extern CO_Data agv_master_Data;


int CoInterfaceInit(MotorController *poMC)
{
    int s32Ret;
    s_BOARD MasterBoard;

    MasterBoard.busname = poMC->busname;
    MasterBoard.baudrate = poMC->baudrate;

    //Initialize The MsgQ For CANopen Server
    struct mq_attr t_mq_attr = {0};
	t_mq_attr.mq_flags = 0; //Block Mode
	t_mq_attr.mq_msgsize = sizeof(TCoIfMsg);
	t_mq_attr.mq_maxmsg = 8;

    mq_unlink(QName);
    mode_t omask;
    omask = umask(0); //返回之前的文件权限默认值，同时修改默认的umask数值为0，这样后面设置文件权限的时候，umask值无影响了（创建文件时，文件权限的计算方式是：umask取反 & 要设定的文件权限）
    QCoIf = mq_open(QName, O_RDWR | O_CREAT | O_NONBLOCK , (S_IRWXU | S_IRWXG | S_IRWXO) /* 777 其他用户可修改*/,  &t_mq_attr);
    umask(omask);
    if (QCoIf < 0)
    {
        printf("q_agv_co_svr create failed with error:%s\n",strerror(errno));
    }
    assert(QCoIf >= 0);
    poMC->QCoIf_ = QCoIf;


    TimerInit();
    //Load The socketcan driver    
    if (LoadCanDriver("/usr/local/lib/libcanfestival_can_socket.so") == NULL)
    {
        printf("Unable to load library:/usr/lib/libcanfestival_can_socket.so\n");
        return -1;
    }

    //Register User Specified Call Back
    agv_master_Data.preOperational = CoMasterPostMasterPreOp;
    agv_master_Data.post_SlaveBootup = CoMasterPostSlaveBootup;

    //Open the CAN Socket

    printf("SIMULATION CAN OPEN!!!\n");

    if(!canOpen(&MasterBoard,&agv_master_Data))
    {
        printf("error open Master Board (%s,%s)\n",MasterBoard.busname, MasterBoard.baudrate);
        return -1;
	}
    StartTimerLoop(&CoMasterInit);

    //Start The Backgroud Timer Fuction
    //为解决odom在两个发布周期给出重复值，以下通过CANopen访问伺服的流程并入main线程
    /*********************************
    pthread_attr_t pthread_attr;
    struct sched_param sched_param;
    pthread_attr_init(&pthread_attr);
    pthread_attr_setinheritsched(&pthread_attr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&pthread_attr, SCHED_OTHER);
    sched_param.sched_priority = 50;
    pthread_attr_setschedparam(&pthread_attr, &sched_param);

    s32Ret = pthread_create(&poMC->thCanSvr,
                &pthread_attr, CoInterfaceTask, (void *)(poMC) );

    assert(s32Ret >= 0);
    *********************************/
    return 0;
}

void CoInterfaceDeInit()
{
    StopTimerLoop(&CoMasterExit);
}
/**
 * @brief 协议栈主站开始运行时的回调
 * @param d CANopen 全局结构 
 * @param nodeid CANopen 伺服子站节点地址 
 * @return void
 */
void CoMasterInit(CO_Data* d, UNS32 id)
{
    /* init */
    setState(&agv_master_Data, Initialisation);
    printf("CoMasterInit\n");
}

/**
 * @brief 协议栈主站终止运行时的回调
 * @param d CANopen 全局结构 
 * @param nodeid CANopen 伺服子站节点地址 
 * @return void
 */
void CoMasterExit(CO_Data* d, UNS32 nodeid)
{
    masterSendNMTstateChange(&agv_master_Data, 1 , NMT_Reset_Node);
    masterSendNMTstateChange(&agv_master_Data, 2 , NMT_Reset_Node);
    setState(&agv_master_Data, Stopped);
}

/**
 * @brief 轮电机伺服上电通知的回调函数
 * @param d CANopen 全局结构 
 * @param nodeid CANopen 伺服子站节点地址 
 * @return void
 */
void CoMasterPostSlaveBootup(CO_Data* d, UNS8 nodeid)
{
    printf("nodeid %d bootup!\n", nodeid);
    fflush(stdout);
}

/**
 * @brief CANopen协议栈进人PreOp状态前的回调
 * @param d CANopen 全局结构 
 * @return void
 */
void CoMasterPostMasterPreOp(CO_Data* d)
{
	printf("Master_preOperational\n");

    masterSendNMTstateChange(&agv_master_Data, 1 , NMT_Reset_Node);
    masterSendNMTstateChange(&agv_master_Data, 2 , NMT_Reset_Node);
}


/**
 * @brief CANopen协议SDO读请求回调
 * @param d CANopen 全局结构 
 * @return void
 */
void CoMasterCheckReadSDO(CO_Data* d, UNS8 nodeid)
{
	UNS32 abortCode;
	UNS32 data=0;
	UNS32 size=64;

    TCoIfMsg tCoIfMsg = {0};
    tCoIfMsg.id = nodeid;

	if(getReadResultNetworkDict(&agv_master_Data, nodeid, &data, &size, &abortCode) != SDO_FINISHED)
    {
        tCoIfMsg.T = TC_SDO_NACK;
        //printf("\nResult : Failed in getting information for slave %2.2x, AbortCode :%4.4x \n", nodeid, abortCode);
    }
	else
    {
        if (size == 1)
        {
            tCoIfMsg.T = TC_R1SDO_ACK;
            tCoIfMsg.B.u8 = (uint8_t)data;
        }
        else if (size == 2)
        {
            tCoIfMsg.T = TC_R2SDO_ACK;
            tCoIfMsg.B.u16 = (uint16_t)data;
        }
        else if (size == 4)
        {
            tCoIfMsg.T = TC_R4SDO_ACK;
            tCoIfMsg.B.u32 = data;
        }
        else
        {
            tCoIfMsg.T = TC_SDO_NACK;
        }
		//printf("\nResult : %x\n", data);
    }


	/* Finalize last SDO transfer with this node */
	closeSDOtransfer(&agv_master_Data, nodeid, SDO_CLIENT);
    //通知CoInterfaceTask处理
    mq_send(QCoIf, (char *)&tCoIfMsg, sizeof(TCoIfMsg), 0);
}
/**
 * @brief CANopen协议SDO写请求回调
 * @param d CANopen 全局结构 
 * @return void
 */
void CoMasterCheckWriteSDO(CO_Data* d, UNS8 nodeid)
{
	UNS32 abortCode;

    TCoIfMsg tCoIfMsg = {0};

    tCoIfMsg.id = nodeid;


	if(getWriteResultNetworkDict(&agv_master_Data, nodeid, &abortCode) != SDO_FINISHED)
	{
        tCoIfMsg.T = TC_SDO_NACK;
        //printf("\nResult : Failed in getting information for slave %2.2x, AbortCode :%4.4x \n", nodeid, abortCode);
    }
	else
    {
        tCoIfMsg.T = TC_WSDO_ACK;
        //printf("\nSend data OK\n");
    }
	/* Finalize last SDO transfer with this node */
	closeSDOtransfer(&agv_master_Data, nodeid, SDO_CLIENT);

    //通知CoInterfaceTask处理
    mq_send(QCoIf, (char *)&tCoIfMsg, sizeof(TCoIfMsg), 0);
}

/**
 * @brief CANopen协议SDO写请求 在CoInterfaceTask中调用，
 * 注意：该方法会调用CANopen的Muxtex方法保护资源，不能在CANopen的任何回调中调用
 * 否则将产生死锁
 * @param d CANopen 全局结构 
 * @return void
 */
int CoMotorResetRequest(MotorController *poMc)
{
    uint8_t u8Ret;
    uint16_t u16Cmd = 0x0086;

    EnterMutex();
    u8Ret = writeNetworkDictCallBack(&agv_master_Data, AGV_LMOTOR_ID, 0x6040, 0, 2, 0, &u16Cmd, CoMasterCheckWriteSDO, 0);
    poMc->LmStatus = (u8Ret == 0 ? ComStatus::SEND : ComStatus::ERROR);
    u8Ret = writeNetworkDictCallBack(&agv_master_Data, AGV_RMOTOR_ID, 0x6040, 0, 2, 0, &u16Cmd, CoMasterCheckWriteSDO, 0);
    poMc->RmStatus = (u8Ret == 0 ? ComStatus::SEND : ComStatus::ERROR);
    LeaveMutex();

    return 0;

}

/**
 * @brief CANopen协议SDO写请求 在CoInterfaceTask中调用，
 * 注意：该方法会调用CANopen的Muxtex方法保护资源，不能在CANopen的任何回调中调用
 * 否则将产生死锁
 * @param d CANopen 全局结构 
 * @return void
 */
int CoMotorEnableRequest(MotorController *poMc)
{
    uint8_t u8Ret;
    uint16_t u16Cmd = 0x002f;

    EnterMutex();
    u8Ret = writeNetworkDictCallBack(&agv_master_Data, AGV_LMOTOR_ID, 0x6040, 0, 2, 0, &u16Cmd, CoMasterCheckWriteSDO, 0);
    poMc->LmStatus = (u8Ret == 0 ? ComStatus::SEND : ComStatus::ERROR);
    u8Ret = writeNetworkDictCallBack(&agv_master_Data, AGV_RMOTOR_ID, 0x6040, 0, 2, 0, &u16Cmd, CoMasterCheckWriteSDO, 0);
    poMc->RmStatus = (u8Ret == 0 ? ComStatus::SEND : ComStatus::ERROR);
    LeaveMutex();

    return 0;

}


/**
 * @brief CANopen协议SDO读编码器请求 在CoInterfaceTask中调用，
 * 注意：该方法会调用CANopen的Muxtex方法保护资源，不能在CANopen的任何回调中调用
 * 否则将产生死锁
 * @param d CANopen 全局结构 
 * @return void
 */
int CoMotorPostionGetRequest(MotorController *poMc)
{
    uint8_t u8Ret;
    //uint16_t u16ResetCmd = 0x002f;

    EnterMutex();
    u8Ret = readNetworkDictCallback(&agv_master_Data, AGV_LMOTOR_ID, 0x6063, 0x00, 0, CoMasterCheckReadSDO, 0);
    poMc->LmStatus = (u8Ret == 0 ? ComStatus::SEND : ComStatus::ERROR);
    u8Ret = readNetworkDictCallback(&agv_master_Data, AGV_RMOTOR_ID, 0x6063, 0x00, 0, CoMasterCheckReadSDO, 0);
    poMc->RmStatus = (u8Ret == 0 ? ComStatus::SEND : ComStatus::ERROR);
    LeaveMutex();

    return 0;

}


/**
 * @brief CANopen协议SDO读编码器请求 在CoInterfaceTask中调用，
 * 注意：该方法会调用CANopen的Muxtex方法保护资源，不能在CANopen的任何回调中调用
 * 否则将产生死锁
 * @param d CANopen 全局结构 
 * @return void
 */
int CoMotorVelocitySetRequest(MotorController *poMc)
{
    uint8_t u8Ret;
    int32_t s32LVCmd;
    int32_t s32RVCmd;

    //todo 读锁定 理论上自然对齐的32位以下的变量赋值是在一个机器周期内完成，
    //无锁操作仅可能导致左右速度的下发出现在前后相邻控制周期，
    s32LVCmd = poMc->s32AgvLV;
    s32RVCmd = poMc->s32AgvRV;

    EnterMutex();
    u8Ret = writeNetworkDictCallBack(&agv_master_Data, AGV_LMOTOR_ID, 0x60FF, 0, 4, 0, &s32LVCmd, CoMasterCheckWriteSDO, 0);
    poMc->LmStatus = (u8Ret == 0 ? ComStatus::SEND : ComStatus::ERROR);
    u8Ret = writeNetworkDictCallBack(&agv_master_Data, AGV_RMOTOR_ID, 0x60FF, 0, 4, 0, &s32RVCmd, CoMasterCheckWriteSDO, 0);
    poMc->RmStatus = (u8Ret == 0 ? ComStatus::SEND : ComStatus::ERROR);
    LeaveMutex();

    return 0;

}

/**
 * @brief 启动电机控制
 * 注意：该方法会调用CANopen的Muxtex方法保护资源，不能在CANopen的任何回调中调用
 * 否则将产生死锁
 * @param d CANopen 全局结构 
 * @return void
 */
int CoMotorCtrlStart()
{
    TCoIfMsg tCoIfMsg = {0};
    tCoIfMsg.T = TC_MC_RESET;
    return mq_send(QCoIf, (char *)&tCoIfMsg, sizeof(TCoIfMsg), 0);
}





/**
 * 废弃不再使用 CANopen接口线程，功能并入Main
 * @brief CANopen接口服务主线程
 * @param d CANopen 全局结构 
 * @return void
void *CoInterfaceTask(void *arg)
{
    int s32Ret;
    TCoIfMsg tCoIfMsg;

    MotorController *poMC = (MotorController *)arg;
    struct timeval t_tmptv;
	fd_set t_fd_read_set;
    int s32_fd_max = QCoIf + 1;

    int s32UsFromHz;
    assert(poMC->s32CtrlFreq > 1);
    s32UsFromHz = floor(1000000.0/poMC->s32CtrlFreq);

    while(1)
    {
        t_tmptv.tv_sec = 0;
        t_tmptv.tv_usec = s32UsFromHz;
        FD_ZERO(&t_fd_read_set);
        FD_SET(QCoIf, &t_fd_read_set);
        s32Ret = select(s32_fd_max, &t_fd_read_set, NULL, NULL, &t_tmptv);
        if (s32Ret < 0)
        {
            printf("file_svr select with err %s\n", strerror(errno));
            sleep(1);
            continue;
        }

        //控制周期边界 触发SDO速度指令发送 推动状态机执行
        else if (s32Ret == 0)
        {
            //printf("Control Period Here...\n");
            poMC->CoIfPeriodCtrl();
            continue;
        }

        //异步SDO响应 接收到SDO
        if (FD_ISSET(QCoIf, &t_fd_read_set))
        {
            memset(&tCoIfMsg, 0, sizeof(TCoIfMsg));

            s32Ret = mq_receive(QCoIf, (char *)&tCoIfMsg,
                        sizeof(TCoIfMsg), NULL);
            if (s32Ret < 0)
            {
                printf("QCoIf Recv Error: %s", strerror(errno));
                continue;
            }

            poMC->CoIfMsgHandler(&tCoIfMsg);

        }


    }

    return NULL;
}

*/