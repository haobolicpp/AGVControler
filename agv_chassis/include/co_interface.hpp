
#ifndef CO_INTERFACE_H
#define CO_INTERFACE_H

#include <stdint.h>

class MotorController;
//CONTROL MSG
#define TC_WSDO_ACK   0x00F0
#define TC_R1SDO_ACK    0x00F1
#define TC_R2SDO_ACK    0x00F2
#define TC_R3SDO_ACK    0x00F3
#define TC_R4SDO_ACK    0x00F4
#define TC_SDO_NACK 0x00FF

#define TC_MC_RESET 0x0001



//#define TC_SERVO_ON 0x0001
//#define TC_SERVO_OFF 0x0002

#define TC_LWHEEL_VEL 0x0010
#define TC_RWHEEL_VEL 0x0020

//DATA FEEDBACK MSG
#define TD_LWHEEL_POSE 0x1010
#define TD_RWHEEL_POSE 0x1020

//ERROR INFOMATION MSG
#define TE_CAN_ERROR 0xF001
#define TE_SERVO_ERROR 0xF002


struct CoIfMsg
{
    uint8_t id;
    uint16_t T;
    union
    {
        uint8_t u8;
        int8_t i8;
        uint16_t u16;
        int16_t i16;
        uint32_t u32;
        int32_t i32;
        uint64_t u64;
        int64_t i64;
    } B;

};

int CoInterfaceInit(MotorController *poMC);
void CoInterfaceDeInit();
/**
 * @brief 协议栈主站开始运行时的回调
 * @param d CANopen 全局结构 
 * @param nodeid CANopen 伺服子站节点地址 
 * @return void
 */
void CoMasterInit(CO_Data* d, UNS32 id);

/**
 * @brief 协议栈主站终止运行时的回调
 * @param d CANopen 全局结构 
 * @param nodeid CANopen 伺服子站节点地址 
 * @return void
 */
void CoMasterExit(CO_Data* d, UNS32 nodeid);

/**
 * @brief 轮电机伺服上电通知的回调函数
 * @param d CANopen 全局结构 
 * @param nodeid CANopen 伺服子站节点地址 
 * @return void
 */
void CoMasterPostSlaveBootup(CO_Data* d, UNS8 nodeid);
/**
 * @brief CANopen协议栈进人PreOp状态前的回调
 * @param d CANopen 全局结构 
 * @return void
 */
void CoMasterPostMasterPreOp(CO_Data* d);


/**
 * @brief CANopen 电机周期控制启动入口
 * @param d CANopen 全局结构 
 * @return void
 */
int CoMotorResetRequest(MotorController *poMc);

/**
 * @brief CANopen 电机使能请求
 * @param d CANopen 全局结构 
 * @return void
 */
int CoMotorEnableRequest(MotorController *poMc);

/**
 * @brief CANopen 读电机编码器请求
 * @param d CANopen 全局结构 
 * @return void
 */
int CoMotorPostionGetRequest(MotorController *poMc);

/**
 * @brief CANopen 速度指令下发请求
 * @param d CANopen 全局结构 
 * @return void
 */
int CoMotorVelocitySetRequest(MotorController *poMc);

/**
 * @brief 启动电机控制
 * @param 
 * @return void
 */
int CoMotorCtrlStart();

/**
 * @brief CANopen接口服务主线程
 * @param 
 * @return void
 */
void *CoInterfaceTask(void *arg);

#endif // CO_INTERFACE_H


