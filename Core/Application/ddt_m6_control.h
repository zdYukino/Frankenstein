/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.


  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef DDT_M6_CONTROL_H
#define DDT_M6_CONTROL_H

#include "main.h"

/*DDT Motor Default 参数*/
#define TORQUE_CONSTANT 0.75f //转矩常数 NM/A
#define DDT_T_MIN (-4.0f)
#define DDT_T_MAX   4.0f

#define DDT_V_MIN (-330)
#define DDT_V_MAX   330
/* CAN send and receive ID */
/*接收到的DM电机的参数结构体*/
/*
* ID 表示控制器的 ID
* err 表示故障码，对应故障类型为：
* bit0——传感器故障
* bit1——过流故障
* bit2——相电流过流；
* bit3——堵转故障
* bit4——过温故障
* Eg:0x02->0b00000010->过流故障
* TOQ 表示电机的转矩力矩信息
* POS 表示电机的单圈位置信息
* RPM 表示电机的转速信息
*/
typedef struct{
    uint8_t id;          //回传信息
    uint8_t mode;        //回传信息
    uint8_t err;         //回传信息
    int16_t int16_toq;   //回传信息
    int16_t int16_rpm;   //回传信息
    uint16_t uint16_pos; //回传信息

    uint16_t uint16_pos_last;   //解码信息
    float angle;                //解码信息
    float toq;                  //解码信息

    uint8_t wheel_init_flag;    //解码信息 里程计
    int32_t  pos_total;         //解码信息 里程计
    float x;                    //解码信息 里程计
}DDT_measure_t;

typedef enum
{
    CURRENT_MODE  = 0x01,
    VELOCITY_MODE = 0x02,
    POSITION_MODE = 0x03,
} DDT_mode_e;

extern DDT_measure_t  DDT_measure[2];
extern uint8_t send_data[10];       //电机控制数据
/* RS485 send and receive ID */

extern void get_ddt_motor_measure(uint8_t *Data, uint8_t length);
extern void DDT_motor_toq_CTRL(UART_HandleTypeDef *huart, uint16_t id, float toq);
extern void DDT_motor_vel_CTRL(UART_HandleTypeDef *huart, uint16_t id, float vel);
extern void DDT_motor_mode_CHANGE(UART_HandleTypeDef *huart, uint16_t id, uint8_t mode);
extern const DDT_measure_t *get_ddt_motor_measure_point(uint8_t i);
#endif
