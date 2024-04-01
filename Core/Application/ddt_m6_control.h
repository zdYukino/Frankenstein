/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             ������CAN�жϽ��պ��������յ������,CAN���ͺ������͵���������Ƶ��.


  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef DDT_M6_CONTROL_H
#define DDT_M6_CONTROL_H

#include "main.h"

/*DDT Motor Default ����*/
#define TORQUE_CONSTANT 0.75f //ת�س��� NM/A
#define DDT_T_MIN (-4.0f)
#define DDT_T_MAX   4.0f

#define DDT_V_MIN (-330)
#define DDT_V_MAX   330
/* CAN send and receive ID */
/*���յ���DM����Ĳ����ṹ��*/
/*
* ID ��ʾ�������� ID
* err ��ʾ�����룬��Ӧ��������Ϊ��
* bit0��������������
* bit1������������
* bit2���������������
* bit3������ת����
* bit4�������¹���
* Eg:0x02->0b00000010->��������
* TOQ ��ʾ�����ת��������Ϣ
* POS ��ʾ����ĵ�Ȧλ����Ϣ
* RPM ��ʾ�����ת����Ϣ
*/
typedef struct{
    uint8_t id;          //�ش���Ϣ
    uint8_t mode;        //�ش���Ϣ
    uint8_t err;         //�ش���Ϣ
    int16_t int16_toq;   //�ش���Ϣ
    int16_t int16_rpm;   //�ش���Ϣ
    uint16_t uint16_pos; //�ش���Ϣ

    uint16_t uint16_pos_last;   //������Ϣ
    float angle;                //������Ϣ
    float toq;                  //������Ϣ

    uint8_t wheel_init_flag;    //������Ϣ ��̼�
    int32_t  pos_total;         //������Ϣ ��̼�
    float x;                    //������Ϣ ��̼�
}DDT_measure_t;

typedef enum
{
    CURRENT_MODE  = 0x01,
    VELOCITY_MODE = 0x02,
    POSITION_MODE = 0x03,
} DDT_mode_e;

extern DDT_measure_t  DDT_measure[2];
extern uint8_t send_data[10];       //�����������
/* RS485 send and receive ID */

extern void get_ddt_motor_measure(uint8_t *Data, uint8_t length);
extern void DDT_motor_toq_CTRL(UART_HandleTypeDef *huart, uint16_t id, float toq);
extern void DDT_motor_vel_CTRL(UART_HandleTypeDef *huart, uint16_t id, float vel);
extern void DDT_motor_mode_CHANGE(UART_HandleTypeDef *huart, uint16_t id, uint8_t mode);
extern const DDT_measure_t *get_ddt_motor_measure_point(uint8_t i);
#endif
