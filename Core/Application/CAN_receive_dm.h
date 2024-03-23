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

/**
  * @brief          float转int 带限幅
  * @param[in]      x：输入值
  * @param[in]      x_min:最小限幅
  * @param[in]      x_max:最大限幅
  * @param[in]      bits:位
  * @retval         返回值
  */
int float_to_uint(float x, float x_min, float x_max, int bits);
/**
  * @brief          int转float 带限幅
  * @param[in]      x_int：输入值
  * @param[in]      x_min:最小限幅
  * @param[in]      x_max:最大限幅
  * @param[in]      bits:位
  * @retval         返回值
  */
float uint_to_float(int x_int, float x_min, float x_max, int bits);
/**
  * @brief          CAN1和CAN2滤波器配置
  * @param[in]      CAN_number：CAN接口数量
  * @retval         None
  */
void CAN_Filter_Init(uint8_t CAN_number);
/**
  * @brief          MIT协议控制DM电机
  * @param[in]      hcan    :CAN接口
  * @param[in]      id      :DM电机ID
  * @param[in]      _pos    :位置设置(rad)
  * @param[in]      _vel    :速度设置(rad/s)
  * @param[in]      _KP     :位置P
  * @param[in]      _KD     :位置D
  * @param[in]      _torq   :力矩
  * @retval         none
  */
void MIT_motor_CTRL(CAN_HandleTypeDef *hcan,uint16_t id, float _pos, float _vel, float _KP, float _KD, float _torq);


/**
  * @brief          返回电机数据指针
  * @param[in]      i: 电机编号,范围[0,3]
  * @retval         电机数据指针
  */
extern const DM_measure_t *get_motor_measure_point(uint8_t i);
/**
  * @brief          使能DM电机
  * @param[in]      hcan: CAN接口
  * @param[in]      id:  DM电机ID（通过上位机设置）
  * @retval         none
  */
void start_motor(CAN_HandleTypeDef* hcan,uint16_t id);
/**
  * @brief          失能DM电机
  * @param[in]      hcan: CAN接口
  * @param[in]      id:  DM电机ID（通过上位机设置）
  * @retval         none
  */
void lock_motor(CAN_HandleTypeDef* hcan,uint16_t id);
/**
  * @brief          设置电机零点
  * @param[in]      hcan: CAN接口
  * @param[in]      id:  DM电机ID（通过上位机设置）
  * @retval         none
  */
void set_zero_motor(CAN_HandleTypeDef *hcan,uint16_t id);

#endif
