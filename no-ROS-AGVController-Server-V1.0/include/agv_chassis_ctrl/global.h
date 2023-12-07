/*
 * @Author: yang.cheng yang.cheng@bgirobot.com
 * @Date: 2023-04-10 08:32:38
 * @LastEditors: yang.cheng yang.cheng@bgirobot.com
 * @LastEditTime: 2023-07-26 15:41:57
 * @FilePath: /agv_controller/include/agv_chassis_ctrl/global.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#ifndef GLOBAL_H
#define GLOBAL_H

//Left Motor Node ID on CAN bus
#define AGV_LMOTOR_ID 2

//Right Motor Node ID on CAN bus
#define AGV_RMOTOR_ID 1

//Control Frequency
#define AGV_MOTOR_CTRL_FREQ 50.0

typedef struct CoIfMsg TCoIfMsg;
class MotorController;


#endif // GLOBAL_H
