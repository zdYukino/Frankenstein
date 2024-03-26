/**
  ****************************(C)ZDYUKINO****************************
  * @file       can_receive.c/h
  * @brief
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V2.0.0     ***-**-2022     ZDYUKINO        加入各类can通信函数 CAN1控制底盘与上层电机通信
                                                CAN2控制sbus数据接收与miss电机通信
                                                带码盘通信函数（未启用）
  *
  @verbatim
  ==============================================================================
  ==============================================================================
  @endverbatim
  ****************************(C)ZDYUKINO****************************
  */

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "main.h"

#define CHASSIS_CAN hcan1
#define BUS_CAN     hcan2
/*DM Motor Default 参数*/
#define P_MIN (-12.5f)
#define P_MAX 12.5f
#define V_MIN (-30.0f)
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN (-12.0f)
#define T_MAX 12.0f
/* CAN send and receive ID */
/*接收到的DM电机的参数结构体*/
/*
* ID 表示控制器的 ID，取 CAN_ID 的低 8 位
* ERR 表示故障，对应故障类型为：
* 8——超压；
* 9——欠压；
* A——过电流；
* B——MOS 过温；
* C——电机线圈过温；
* D——通讯丢失；
* E——过载；
* POS 表示电机的位置信息
* VEL 表示电机的速度信息
* T 表示电机的扭矩信息
* T_MOS 表示驱动上 MOS 的平均温度，单位℃
* T_Rotor 表示电机内部线圈的平均温度，单位℃
*/
typedef struct{
    uint8_t id;
    uint8_t state;
    int int_p;
    int int_v;
    int int_t;
    float pos;
    float vel;
    float toq;
    float T_mos;
    float T_coil;
}DM_measure_t;
extern DM_measure_t  DM_Motor_measure[8];

typedef enum
{
    CAN_DM_M1_ID = 0x01,
    CAN_DM_M2_ID = 0x02,
    CAN_DM_M3_ID = 0x03,
    CAN_DM_M4_ID = 0x04,

    CAN_DM_M5_ID = 0x05,
    CAN_DM_M6_ID = 0x06,
    CAN_DM_M7_ID = 0x07,
    CAN_DM_M8_ID = 0x08,

    MASTER_ID = 0x00,
} can_msg_id_e;

/* CAN send and receive ID */

extern void CAN_Filter_Init(uint8_t CAN_number);
extern void MIT_motor_CTRL(CAN_HandleTypeDef *hcan,uint16_t id, float _pos, float _vel, float _KP, float _KD, float _torq);
extern const DM_measure_t *get_motor_measure_point(uint8_t i);
extern void start_motor(CAN_HandleTypeDef* hcan,uint16_t id);
extern void lock_motor(CAN_HandleTypeDef* hcan,uint16_t id);
extern void set_zero_motor(CAN_HandleTypeDef *hcan,uint16_t id);

#endif
