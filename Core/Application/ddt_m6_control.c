/**
  ****************************(C)ZDYUKINO****************************
  * @file       ddt_m6_control.c/h
  * @brief      这里是本末电机M6驱动代码
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

DDT_measure_t DDT_measure[2];//电机数据结构体定义
uint8_t send_data[10];       //电机控制数据
/**
  * @brief          CRC_MAXIM校验
  * @param[in]      data:校验数据
  * @param[in]      len: 数据长度
  * @retval[out]    return:校验值
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
  * @brief          DDT电机回传参数转换
  * @param[in]      Data：数据指针
  * @param[in]      length：数据长度
  * @retval         None
  */
void get_ddt_motor_measure(uint8_t *Data, uint8_t length)
{
    if(length != 10) return;    //长度校验
    if(Data[9]!=crc8_MAXIM(Data,9)) return; //CRC校验
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
        if(DDT_measure[ID-1].int16_rpm <= 0)  //正转
        {
            if (DDT_measure[ID-1].uint16_pos_last - DDT_measure[ID-1].uint16_pos >= 16384)
                DDT_measure[ID-1].pos_total += 32767 - DDT_measure[ID-1].uint16_pos_last + DDT_measure[ID-1].uint16_pos;
            else
                DDT_measure[ID-1].pos_total += DDT_measure[ID-1].uint16_pos - DDT_measure[ID-1].uint16_pos_last;
        }
        else                                //反转
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
  * @brief          本末电机电流控制
  * @param[in]      huart   :RS485接口
  * @param[in]      toq   :力矩 -2NM~2NM
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
  * @brief          本末电机电流控制
  * @param[in]      huart   :RS485接口
  * @param[in]      toq   :力矩 -2NM~2NM
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
  * @brief          返回电机数据指针
  * @param[in]      i: 电机编号,范围[0,2]
  * @retval         电机数据指针
  */
const DDT_measure_t *get_ddt_motor_measure_point(uint8_t i)
{
    return &DDT_measure[(i&0x03)];//0011B
}
/**
  * @brief          串口DMA发送完成中断
  * @param[in]      huart: 串口指针
  * @retval         电机数据指针
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart == &huart2)
        HAL_GPIO_WritePin(COM_485_PORT,COM_485_PIN,GPIO_PIN_RESET);//关闭RS485发送，转为接收模式
}