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

/**
  * @brief          floatתint ���޷�
  * @param[in]      x������ֵ
  * @param[in]      x_min:��С�޷�
  * @param[in]      x_max:����޷�
  * @param[in]      bits:λ
  * @retval         ����ֵ
  */
int float_to_uint(float x, float x_min, float x_max, int bits);
/**
  * @brief          intתfloat ���޷�
  * @param[in]      x_int������ֵ
  * @param[in]      x_min:��С�޷�
  * @param[in]      x_max:����޷�
  * @param[in]      bits:λ
  * @retval         ����ֵ
  */
float uint_to_float(int x_int, float x_min, float x_max, int bits);
/**
  * @brief          CAN1��CAN2�˲�������
  * @param[in]      CAN_number��CAN�ӿ�����
  * @retval         None
  */
void CAN_Filter_Init(uint8_t CAN_number);
/**
  * @brief          MITЭ�����DM���
  * @param[in]      hcan    :CAN�ӿ�
  * @param[in]      id      :DM���ID
  * @param[in]      _pos    :λ������(rad)
  * @param[in]      _vel    :�ٶ�����(rad/s)
  * @param[in]      _KP     :λ��P
  * @param[in]      _KD     :λ��D
  * @param[in]      _torq   :����
  * @retval         none
  */
void MIT_motor_CTRL(CAN_HandleTypeDef *hcan,uint16_t id, float _pos, float _vel, float _KP, float _KD, float _torq);


/**
  * @brief          ���ص������ָ��
  * @param[in]      i: ������,��Χ[0,3]
  * @retval         �������ָ��
  */
extern const DM_measure_t *get_motor_measure_point(uint8_t i);
/**
  * @brief          ʹ��DM���
  * @param[in]      hcan: CAN�ӿ�
  * @param[in]      id:  DM���ID��ͨ����λ�����ã�
  * @retval         none
  */
void start_motor(CAN_HandleTypeDef* hcan,uint16_t id);
/**
  * @brief          ʧ��DM���
  * @param[in]      hcan: CAN�ӿ�
  * @param[in]      id:  DM���ID��ͨ����λ�����ã�
  * @retval         none
  */
void lock_motor(CAN_HandleTypeDef* hcan,uint16_t id);
/**
  * @brief          ���õ�����
  * @param[in]      hcan: CAN�ӿ�
  * @param[in]      id:  DM���ID��ͨ����λ�����ã�
  * @retval         none
  */
void set_zero_motor(CAN_HandleTypeDef *hcan,uint16_t id);

#endif
