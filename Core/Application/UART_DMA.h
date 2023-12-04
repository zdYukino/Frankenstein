/**
  ****************************(C)SWJTU_ROBOTCON2022****************************
  * @file       uart_dma.c/h
  * @brief      ������UART_DMA���պ���
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     1-12-2022       ZDYukino        1. done
  *
  @verbatim
  ==============================================================================
  ==============================================================================
  @endverbatim
  ****************************(C)SWJTU_ROBOTCON2023****************************
  **/

#ifndef UART_DMA_H
#define UART_DMA_H

#include "main.h"
#include "usart.h"

//���建�������С
#define buffer_receive_1_length 50
#define buffer_receive_2_length 50
#define buffer_receive_3_length 50
#define buffer_receive_4_length 50
#define buffer_receive_5_length 50
#define buffer_receive_6_length 50

extern uint8_t buffer_receive_1[buffer_receive_1_length];//��������ȫ�ֶ���
extern uint8_t buffer_receive_2[buffer_receive_2_length];//��������ȫ�ֶ���
extern uint8_t buffer_receive_3[buffer_receive_3_length];//��������ȫ�ֶ���
extern uint8_t buffer_receive_4[buffer_receive_4_length];//��������ȫ�ֶ���
extern uint8_t buffer_receive_5[buffer_receive_5_length];//��������ȫ�ֶ���
extern uint8_t buffer_receive_6[buffer_receive_6_length];//��������ȫ�ֶ���

/**
  * @brief          ��ʼ������DMA����
  * @param[in]      UART�ӿ�
  * @param[in]      ��������->�Ƽ�ʹ���Ѷ�������
  * @param[in]      ����  ��1-128��
  * @retval         none
  */
extern void UART_DMA_Receive_init(UART_HandleTypeDef *usart, uint8_t *buffer, uint8_t length);
/**
  * @brief          ����DMA�����жϺ���->���롶USER CODE BEGIN USARTX_IRQn 1�� ��
  * @param[in]      UART�ӿ�
  * @param[in]      UART DMA�ӿ�
  * @param[in]      ��������->�Ƽ�ʹ���Ѷ�������
  * @param[in]      ����  ��1-128��
  * @retval         none
  */
extern void UART_DMA_Receive_IT(UART_HandleTypeDef *usart, DMA_HandleTypeDef *DMA, uint8_t *buffer, uint8_t length);
#endif
