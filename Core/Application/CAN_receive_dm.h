/**
  ****************************(C)ZDYUKINO****************************
  * @file       can_receive.c/h
  * @brief
  *             ������CAN�жϽ��պ��������յ������,CAN���ͺ������͵���������Ƶ��.
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V2.0.0     ***-**-2022     ZDYUKINO        �������canͨ�ź��� CAN1���Ƶ������ϲ���ͨ��
                                                CAN2����sbus���ݽ�����miss���ͨ��
                                                ������ͨ�ź�����δ���ã�
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
/*DM Motor Default ����*/
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
/*���յ���DM����Ĳ����ṹ��*/
/*
* ID ��ʾ�������� ID��ȡ CAN_ID �ĵ� 8 λ
* ERR ��ʾ���ϣ���Ӧ��������Ϊ��
* 8������ѹ��
* 9����Ƿѹ��
* A������������
* B����MOS ���£�
* C���������Ȧ���£�
* D����ͨѶ��ʧ��
* E�������أ�
* POS ��ʾ�����λ����Ϣ
* VEL ��ʾ������ٶ���Ϣ
* T ��ʾ�����Ť����Ϣ
* T_MOS ��ʾ������ MOS ��ƽ���¶ȣ���λ��
* T_Rotor ��ʾ����ڲ���Ȧ��ƽ���¶ȣ���λ��
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
