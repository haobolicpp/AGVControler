
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
#include "global.h"
#include "co_interface.h"
#include "motor_controller.h"



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
        printf("ERROR: q_agv_co_svr create failed with error:%s\n",strerror(errno));
    }
    assert(QCoIf >= 0);
    poMC->QCoIf_ = QCoIf;


    TimerInit();
    //Load The socketcan driver    
    if (LoadCanDriver("/usr/local/lib/libcanfestival_can_socket.so") == NULL)
    {
        printf("ERROR: Unable to load library:/usr/lib/libcanfestival_can_socket.so\n");
        return -1;
    }

    //Register User Specified Call Back
    agv_master_Data.preOperational = CoMasterPostMasterPreOp;
    agv_master_Data.post_SlaveBootup = CoMasterPostSlaveBootup;

    if(!canOpen(&MasterBoard,&agv_master_Data))
    {
        printf("ERROR: Canopen Communication (%s,%s)\n",MasterBoard.busname, MasterBoard.baudrate);
        return -1;
	}
    StartTimerLoop(&CoMasterInit);

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
    printf("INFO: Canopen Master Init...\n");
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
    printf("INFO: Canopen Slave Node ID %d BootUp!\n", nodeid);
    fflush(stdout);
}

/**
 * @brief CANopen协议栈进人PreOp状态前的回调
 * @param d CANopen 全局结构 
 * @return void
 */
void CoMasterPostMasterPreOp(CO_Data* d)
{
	printf("INFO: Canopen Master Enter PreOP...\n");

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

//理论上自然对齐的32位以下的变量赋值是在一个机器周期内完成，
//对控制量进行一阶平滑滤波后输出------------------------
    double LvOut = poMc->SmoothFilterLv(poMc->fAgvLV);
    double RvOut = poMc->SmoothFilterRv(poMc->fAgvRV);
    s32LVCmd = floor(LvOut);
    s32RVCmd = floor(RvOut);

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