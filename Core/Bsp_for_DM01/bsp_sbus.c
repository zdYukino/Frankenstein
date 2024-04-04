/**
  ****************************(C)ZDYUKINO***************************
  * @file       bsp_delay.c/h
  * @brief      DWT延时函数
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     11-3-2024       ZDYukino        1. done
  *  https://blog.csdn.net/w237838/article/details/134771598
  @verbatim
  ===================================================================
  ===================================================================
  @endverbatim
  ****************************(C)ZDYUKINO****************************
  **/
#include "bsp_sbus.h"
#include "main.h"

uint16_t sbus_channel[16];
/**
  * @brief          SUBS解码
  * @param[out]     校验后原始数据
  * @retval         none
*/
void SBUS_trans(const uint8_t *buffer, uint8_t length)
{
    if(buffer[0] == 0x0F && buffer[24] == 0x00 && length == 25)
    {
        sbus_channel[0] = ((int16_t) buffer[1] >> 0 | ((int16_t) buffer[3 - 1] << 8)) & 0x07FF;
        sbus_channel[1] = ((int16_t) buffer[2] >> 3 | ((int16_t) buffer[4 - 1] << 5)) & 0x07FF;
        sbus_channel[2] =
                ((int16_t) buffer[3] >> 6 | ((int16_t) buffer[4] << 2) | (int16_t) buffer[5] << 10) & 0x07FF;
        sbus_channel[3] = ((int16_t) buffer[5] >> 1 | ((int16_t) buffer[6] << 7)) & 0x07FF;
        sbus_channel[4] = ((int16_t) buffer[6] >> 4 | ((int16_t) buffer[7] << 4)) & 0x07FF;
        sbus_channel[5] =
                ((int16_t) buffer[7] >> 7 | ((int16_t) buffer[8] << 1) | (int16_t) buffer[9] << 9) & 0x07FF;
        sbus_channel[6] = ((int16_t) buffer[9] >> 2 | ((int16_t) buffer[10] << 6))  & 0x07FF;
        sbus_channel[7] = ((int16_t) buffer[10] >> 5 | ((int16_t) buffer[11] << 3)) & 0x07FF;
        sbus_channel[8] = ((int16_t) buffer[12] << 0 | ((int16_t) buffer[13] << 8)) & 0x07FF;
        sbus_channel[9] = ((int16_t) buffer[13] >> 3 | ((int16_t) buffer[14] << 5)) & 0x07FF;
        sbus_channel[10] =
                ((int16_t) buffer[14] >> 6 | ((int16_t) buffer[15] << 2) | (int16_t) buffer[16] << 10) & 0x07FF;
        sbus_channel[11] = ((int16_t) buffer[16] >> 1 | ((int16_t) buffer[17] << 7)) & 0x07FF;
        sbus_channel[12] = ((int16_t) buffer[17] >> 4 | ((int16_t) buffer[18] << 4)) & 0x07FF;
        sbus_channel[13] =
                ((int16_t) buffer[18] >> 7 | ((int16_t) buffer[19] << 1) | (int16_t) buffer[20] << 9) & 0x07FF;
        sbus_channel[14] = ((int16_t) buffer[20] >> 2 | ((int16_t) buffer[21] << 6)) & 0x07FF;
        sbus_channel[15] = ((int16_t) buffer[21] >> 5 | ((int16_t) buffer[22] << 3)) & 0x07FF;
    }
    else return;
}