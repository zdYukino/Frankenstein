/**
  ****************************(C)ZDYUKINO***************************
  * @file       uart_dma.c/h
  * @brief      这里是UART_DMA接收函数
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     1-12-2022       ZDYukino        1. done
  *
  @verbatim
  ===================================================================
  ===================================================================
  @endverbatim
  ****************************(C)ZDYUKINO****************************
  **/

#include "main.h"
#include "usart.h"
#include "uart_dma.h"
#include "string.h"
#include "vofa.h"
#include "ddt_m6_control.h"
#include "bsp_sbus.h"

/*缓存数组预定义*/
uint8_t buffer_receive_1[buffer_receive_1_length];
uint8_t buffer_receive_2[buffer_receive_2_length];
uint8_t buffer_receive_3[buffer_receive_3_length];
uint8_t buffer_receive_4[buffer_receive_4_length];
uint8_t buffer_receive_5[buffer_receive_5_length];
uint8_t buffer_receive_6[buffer_receive_6_length];

uint16_t VL_100_distance;

static union
{
    uint8_t data[24];
    float ActVal[6];
}posture;
/**
  * @brief          UART1-6中断接收服务函数
  * @param[in]      接收数组
  * @param[in]      长度值【0-128】
  * @retval         none
  */
static void UART1_Receive_Serve(uint8_t *buffer, uint8_t length);
static void UART2_Receive_Serve(uint8_t *buffer, uint8_t length);
static void UART3_Receive_Serve(uint8_t *buffer, uint8_t length);
static void UART4_Receive_Serve(uint8_t *buffer, uint8_t length);
static void UART5_Receive_Serve(uint8_t *buffer, uint8_t length);
static void UART6_Receive_Serve(uint8_t *buffer, uint8_t length);

/**
  * @brief          初始化串口DMA接收
  * @param[in]      UART接口
  * @param[in]      缓存数组->推荐使用已定义数组
  * @param[in]      长度  【1-128】
  * @retval         none
  */
void UART_DMA_Receive_init(UART_HandleTypeDef *usart, uint8_t *buffer, uint8_t length)
{
  __HAL_UART_ENABLE_IT(usart,UART_IT_IDLE);
  HAL_UART_Receive_DMA(usart,buffer,length);//打开DMA接收
}
/**
  * @brief          串口DMA接收中断函数->放入《USER CODE BEGIN USARTX_IRQn 1》 中
  * @param[in]      UART接口
  * @param[in]      UART DMA接口
  * @param[in]      缓存数组->推荐使用已定义数组
  * @param[in]      长度  【1-128】
  * @retval         none
  */
void UART_DMA_Receive_IT(UART_HandleTypeDef *usart, DMA_HandleTypeDef *DMA, uint8_t *buffer, uint8_t length)
{
 if(__HAL_UART_GET_FLAG(usart, UART_FLAG_IDLE) == 1)
 {
     __HAL_UART_CLEAR_IDLEFLAG(usart);  
     HAL_UART_DMAStop(usart);
     uint8_t real_length = length - __HAL_DMA_GET_COUNTER(DMA);

//     if(usart == &huart1)      UART1_Receive_Serve(buffer, real_length);//选择解码程序
     if(usart == &huart2) UART2_Receive_Serve(buffer, real_length);//选择解码程序
     else if(usart == &huart3) UART3_Receive_Serve(buffer, real_length);//选择解码程序
//     else if(usart == &huart4) UART4_Receive_Serve(buffer, real_length);//选择解码程序
     else if(usart == &huart5) UART5_Receive_Serve(buffer, real_length);//选择解码程序
//   else if(usart == &huart6) UART6_Receive_Serve(buffer, real_length);//选择解码程序
     memset(buffer,0,real_length);
	 HAL_UART_Receive_DMA(usart, buffer, length);//重新打开DMA接收
 }
}
/**
  * @brief          串口异常的处理
  * @param[in]      UART接口
  * @retval         none
  */
void HAL_UART_ErrorCallback (UART_HandleTypeDef *huart)
{
	uint32_t data;
    __HAL_UNLOCK(huart);
 
	if(__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE) != RESET)
	{
		__HAL_UART_CLEAR_FLAG(huart, UART_FLAG_ORE);      		//清除溢出中断
		data = huart->Instance->SR;
		data = huart->Instance->DR;
		__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
	}
}
//UART1中断接收函数
static void UART1_Receive_Serve(uint8_t *buffer, uint8_t length)
{
}
//UART2中断接收函数
static void UART2_Receive_Serve(uint8_t *buffer, uint8_t length)
{
    get_ddt_motor_measure(buffer, length);
//    HAL_UART_Transmit(&huart5,buffer,length,0xff);
}
//UART3中断接收函数
static void UART3_Receive_Serve(uint8_t *buffer, uint8_t length)
{
//    HAL_UART_Transmit(&huart5,buffer,length,0xff);
      SBUS_trans(buffer, length);
}
//UART4中断接收函数
static void UART4_Receive_Serve(uint8_t *buffer, uint8_t length)
{
}
//UART5中断接收函数
static void UART5_Receive_Serve(uint8_t *buffer, uint8_t length)
{
      Vofa_UART_Receive(buffer,length);
}
//UART6中断接收函数
static void UART6_Receive_Serve(uint8_t *buffer, uint8_t length)
{
}
