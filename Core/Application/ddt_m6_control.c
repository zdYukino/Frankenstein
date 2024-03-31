/**
  ****************************(C)ZDYUKINO****************************
  * @file       ddt_m6_control.c/h
  * @brief      �����Ǳ�ĩ���M6��������
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Mar-26-2024     ZDYukino        1. done
  *
  @verbatim
  ===================================================================
  ===================================================================
  @endverbatim
  ****************************(C)ZDYUKINO****************************
  */

#include <math.h>
#include "ddt_m6_control.h"
#include "main.h"
#include "usart.h"
#include "lqr_wbr.h"
#include "vofa_setting.h"

#define COM_485_PORT RS485_DIR2_GPIO_Port
#define COM_485_PIN  RS485_DIR2_Pin

uint8_t wheel_init_flag[2] = {0};

DDT_measure_t DDT_measure[2];//������ݽṹ�嶨��
uint8_t send_data[10];       //�����������
/**
  * @brief          CRC_MAXIMУ��
  * @param[in]      data:У������
  * @param[in]      len: ���ݳ���
  * @retval[out]    return:У��ֵ
  */
static uint8_t crc8_MAXIM(uint8_t *data, uint8_t len)
{
    uint8_t crc, i;
    crc = 0x00;
    while(len--)
    {
        crc ^= *data++;
        for(i = 0;i < 8;i++)
        {
            if(crc & 0x01) crc = (crc >> 1) ^ 0x8c;
            else crc >>= 1;
        }
    }
    return crc;
}
/**
  * @brief          DDT����ش�����ת��
  * @param[in]      Data������ָ��
  * @param[in]      length�����ݳ���
  * @retval         None
  */
void get_ddt_motor_measure(uint8_t *Data, uint8_t length)
{
    if(length != 10) return;    //����У��
    if(Data[9]!=crc8_MAXIM(Data,9)) return; //CRCУ��
    uint8_t ID = Data[0];
    DDT_measure[ID-1].mode = Data[1];
    DDT_measure[ID-1].int16_toq = Data[2]<<8 | Data[3];
    DDT_measure[ID-1].int16_rpm = Data[4]<<8 | Data[5];
    DDT_measure[ID-1].uint16_pos_last = DDT_measure[ID-1].uint16_pos;
    DDT_measure[ID-1].uint16_pos= Data[6]<<8 | Data[7];
    DDT_measure[ID-1].err = Data[8];
    DDT_measure[ID-1].toq = (float)DDT_measure[ID-1].int16_toq*8.0f/32767.0f*TORQUE_CONSTANT;
    DDT_measure[ID-1].angle = (float)DDT_measure[ID-1].uint16_pos*360.0f/32767.0f;

    if( wheel_init_flag[ID-1] == 0)
    {
        DDT_measure[ID-1].x = 0;
        DDT_measure[ID-1].uint16_pos_last = DDT_measure[ID-1].uint16_pos;
        wheel_init_flag[ID-1]= 1;
    }
    else
    {
        if(DDT_measure[ID-1].int16_rpm <= 0)  //��ת
        {
            if (DDT_measure[ID-1].uint16_pos_last - DDT_measure[ID-1].uint16_pos >= 16384)
                DDT_measure[ID-1].pos_total += 32767 - DDT_measure[ID-1].uint16_pos_last + DDT_measure[ID-1].uint16_pos;
            else
                DDT_measure[ID-1].pos_total += DDT_measure[ID-1].uint16_pos - DDT_measure[ID-1].uint16_pos_last;
        }
        else                                //��ת
        {
            if(DDT_measure[ID-1].uint16_pos - DDT_measure[ID-1].uint16_pos_last >= 16384)
                DDT_measure[ID-1].pos_total -= 32767 - DDT_measure[ID-1].uint16_pos + DDT_measure[ID-1].uint16_pos_last;
            else
                DDT_measure[ID-1].pos_total -= DDT_measure[ID-1].uint16_pos_last - DDT_measure[ID-1].uint16_pos;
        }
        DDT_measure[ID-1].x = (float) DDT_measure[ID-1].pos_total / 32767 * (float)M_PI * WHEEl_D;
    }
}

/**
  * @brief          ��ĩ�����������
  * @param[in]      huart   :RS485�ӿ�
  * @param[in]      toq   :���� -2NM~2NM
  * @retval         none
  */
void DDT_motor_toq_CTRL(UART_HandleTypeDef *huart, uint16_t id, float toq)
{
    float max_toq = toq;
    if(toq>DDT_T_MAX) max_toq = DDT_T_MAX;
    else if(toq<DDT_T_MIN) max_toq = DDT_T_MIN;
    int16_t current_tmp = (int16_t)(max_toq/TORQUE_CONSTANT/8*32767.0f);

    send_data[0] = id;
    send_data[1] = 0x64;
    send_data[2] = current_tmp>>8;
    send_data[3] = current_tmp;
    send_data[4] = 0x00;
    send_data[5] = 0x00;
    send_data[6] = 0x00;
    send_data[7] = 0x00;
    send_data[8] = 0x00;
    send_data[9] = crc8_MAXIM(send_data,9);

    HAL_GPIO_WritePin(COM_485_PORT,COM_485_PIN,GPIO_PIN_SET);
    HAL_UART_Transmit_DMA(&huart2,send_data,10);
}

/**
  * @brief          ��ĩ�����������
  * @param[in]      huart   :RS485�ӿ�
  * @param[in]      toq   :���� -2NM~2NM
  * @retval         none
  */
void DDT_motor_vel_CTRL(UART_HandleTypeDef *huart, uint16_t id, float vel)
{
    float max_vel = vel;
//    if(toq>DDT_T_MAX) max_toq = DDT_T_MAX;
//    else if(toq<DDT_T_MIN) max_toq = DDT_T_MIN;
//    else max_toq = toq;
    int16_t current_tmp = max_vel*330.0f;

    send_data[0] = id;
    send_data[1] = 0x64;
    send_data[2] = current_tmp>>8;
    send_data[3] = current_tmp;
    send_data[4] = 0x00;
    send_data[5] = 0x00;
    send_data[6] = 0x00;
    send_data[7] = 0x00;
    send_data[8] = 0x00;
    send_data[9] = crc8_MAXIM(send_data,9);

    HAL_GPIO_WritePin(COM_485_PORT,COM_485_PIN,GPIO_PIN_SET);
    HAL_UART_Transmit_DMA(&huart2,send_data,10);
}
/**
  * @brief          ���ص������ָ��
  * @param[in]      i: ������,��Χ[0,2]
  * @retval         �������ָ��
  */
const DDT_measure_t *get_ddt_motor_measure_point(uint8_t i)
{
    return &DDT_measure[(i&0x03)];//0011B
}
/**
  * @brief          ����DMA��������ж�
  * @param[in]      huart: ����ָ��
  * @retval         �������ָ��
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart == &huart2)
        HAL_GPIO_WritePin(COM_485_PORT,COM_485_PIN,GPIO_PIN_RESET);//�ر�RS485���ͣ�תΪ����ģʽ
}