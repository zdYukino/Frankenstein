/**
 ******************************************************************************
 * @file    vofa.h
 * @author  Sesame
 * @version V1.1
 * @date    2022.03.02
 * @brief   VOFA发送接收程序
 ******************************************************************************
 * @attention
 *
 *
 *
 ******************************************************************************
 */
#ifndef _VOFA_H_
#define _VOFA_H_
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "vofa_setting.h"
//#include "usbd_cdc_if.h"

union VOFA_DataConvertTypeDef
{
  float   DataFloat;
  uint8_t DataChar[4];
};
typedef struct
{
  uint8_t res;
  uint8_t lastres;
  uint8_t cnt;
  uint8_t timeout;
  uint8_t Buff[VOFAClearLen];
} VOFA_RxTypedef;

void Vofa_UART_Receive(const uint8_t *buffer, uint8_t len);
void Vofa_FRAME_Handler(VOFA_RxTypedef uart);
void Vofa_Timeout(void);
void Vofa_Uart_Transmit(UART_HandleTypeDef *huart);
void Vofa_Slider_Handler(uint8_t Num);
void Vofa_Button_Handler(uint8_t Num);
void Vofa_Key_Handler(uint8_t Num);
void Vofa_Bar_Handler(uint8_t Num);
#endif
