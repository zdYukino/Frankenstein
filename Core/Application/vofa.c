#include "vofa.h"
#include "vofa_setting.h"
#include "string.h"
#include "cmsis_os.h"
#include "Attitude.h"
#include "CAN_receive_dm.h"
#include "lqr_wbr.h"
#include "UART_DMA.h"
#include "ddt_m6_control.h"
#include "bsp_sbus.h"

float  VofaData[16] = {0}; //定义控件变量

#define TX_LENGTH 20                //定义需要查看变量数量
float  tempFloat[TX_LENGTH] = {0};  //定义发送变量（just float模式）
uint8_t tempData[TX_LENGTH*4+4];    //DMA发送需为全局变量 大小为len=num*4+4

VOFA_RxTypedef Vofa_RX;
/**
  * @brief Function FREERTOS VOFA发送调试信息
  * @param argument: Not used
  * @retval none
  */
void VofaOutputTask(void const * argument)
{
    UART_DMA_Receive_init(&huart2, buffer_receive_2, buffer_receive_2_length);
    UART_DMA_Receive_init(&huart3, buffer_receive_3, buffer_receive_3_length);
    UART_DMA_Receive_init(&huart5, buffer_receive_5, buffer_receive_5_length);
    /* Infinite loop */
    for(;;)
    {
        tempFloat[0] =  lqr_data_L.vmc_data.phi1;
        tempFloat[1] =  lqr_data_L.vmc_data.phi4;
        tempFloat[2] =  lqr_data_R.vmc_data.phi1;
        tempFloat[3] =  lqr_data_R.vmc_data.phi4;
        tempFloat[4] =  lqr_data_L.theta;
        tempFloat[5] =  lqr_data_L.d_theta;
        tempFloat[6] =  lqr_data_L.x;
        tempFloat[7] =  lqr_data_L.d_x;
        tempFloat[8] =  lqr_data_L.phi;
        tempFloat[9] =  lqr_data_L.d_phi;
        tempFloat[10] = lqr_data_L.wheel_motor_data->x;
        tempFloat[11] = lqr_data_R.wheel_motor_data->x;
        tempFloat[12] = lqr_data_L.length_set;
        tempFloat[13] = lqr_data_L.length_now;
        tempFloat[14] = wbr_control_data.speed_set;
        tempFloat[15] = lqr_data_L.d_x;
        tempFloat[16] = lqr_data_L.x;
        tempFloat[17] = wbr_control_data.height_set;
        tempFloat[18] = sbus_channel[3];
        //tempFloat[19] = VofaData[1];
        Vofa_Uart_Transmit(&huart5);
        osDelay(5);
    }
}

/**
 * @brief  VOFA发送函数
 *         通过此函数发送数据到VOFA
 * @param  huart：要通过VOFA发送的串口
 * @param  num：  要通过VOFA发送的数据个数
 * @retval *
 */
void Vofa_Uart_Transmit(UART_HandleTypeDef *huart)
{
    uint16_t len = TX_LENGTH*4+4;
    memcpy(tempData, (uint8_t *)tempFloat, len-4);
    tempData[len-4] = 0x00;
    tempData[len-3] = 0x00;
    tempData[len-2] = 0x80;
    tempData[len-1] = 0x7f;
    #ifdef USB_TRANSMIT
        CDC_Transmit_FS((uint8_t *)tempData, len);
    #else
        HAL_UART_Transmit_DMA(huart, (uint8_t *)tempData, len);
    #endif
}

/**
 * @brief  串口超时清空接收
 *         须在定时器中定时调用，超时时间为：VOFAOutTime*一次中断时间
 * @param  *
 * @retval *
 */
void Vofa_Timeout(void)
{
    if (Vofa_RX.timeout == 0)
    {
        uint8_t i;
        for (i = 0; i < VOFAClearLen; i++)
        {
            Vofa_RX.Buff[i] = 0;
        }
        Vofa_RX.cnt = 0;
        Vofa_RX.lastres = 0;
        Vofa_RX.res = 0;
    }
    else
    {
        Vofa_RX.timeout--;
    }
}

/**
 * @brief  VOFA帧处理函数
 *         通过此函数处理接收到的VOFA帧
 * @param  data：要通过VOFA发送的数据
 * @retval *
 */
void Vofa_FRAME_Handler(VOFA_RxTypedef uart)
{

    //uint8_t i;
    uint8_t FirstNum, LastNum;
    if (Vofa_RX.Buff[0] == 0xfe && Vofa_RX.Buff[1] == 0xef) //验证首帧
    {
        FirstNum = (Vofa_RX.Buff[2] & 0xf0) >> 4;
        LastNum = Vofa_RX.Buff[2] & 0x0f;
        switch (FirstNum)
        {
        case 0:
            Vofa_Slider_Handler(LastNum); //滑块命令处理
            break;
        case 1:
            Vofa_Button_Handler(LastNum); //按钮命令处理
            break;
        case 2:
            Vofa_Key_Handler(LastNum); //按键命令处理
            break;
        case 3:
            Vofa_Bar_Handler(LastNum); //摇杆命令处理
            break;
        }
    }
}

/**
 * @brief  串口接收函数
 * @param  res：串口接收到的数据
 * @retval *
 */
void Vofa_UART_Receive(const uint8_t *buffer, uint8_t len)
{
	for(uint8_t i=0;i<len;i++)
    {
        Vofa_RX.res = buffer[i];
        Vofa_RX.Buff[Vofa_RX.cnt++] = Vofa_RX.res;
        Vofa_RX.timeout = VOFAClearTime;
        if (Vofa_RX.lastres == 0xed && Vofa_RX.res == 0xde) //接收到尾帧0xed 0xde，进行帧处理
        {		
            Vofa_FRAME_Handler(Vofa_RX); //串口帧处理，清空串口计数
            Vofa_RX.lastres = 0;
            Vofa_RX.cnt = 0;
        }
        Vofa_RX.lastres = Vofa_RX.res;  //将串口接收数据暂存用于辨识尾帧
        if (Vofa_RX.cnt > VOFAClearLen) //超长丢弃
        {
            for (Vofa_RX.cnt = 0; Vofa_RX.cnt < VOFAClearLen; Vofa_RX.cnt++)
            {
                Vofa_RX.Buff[Vofa_RX.cnt] = 0;
            }
    
            Vofa_RX.lastres = 0;
            Vofa_RX.cnt = 0;
        }
     }
}
/**
 * @brief  滑块命令处理
 * @param  命令号
 * @retval *
 */
void Vofa_Slider_Handler(uint8_t Num)
{
    union VOFA_DataConvertTypeDef DataConvert;
    uint8_t i;
    for (i = 0; i < 4; i++)
    {
        DataConvert.DataChar[i] = Vofa_RX.Buff[3 + i];
    }
    switch (Num)
    {
#ifdef UseVOFASlider1
    case 1:
        Vofa_Slider1 = DataConvert.DataFloat;
        break;
#endif
#ifdef UseVOFASlider2
    case 2:
        Vofa_Slider2 = DataConvert.DataFloat;
        break;
#endif
#ifdef UseVOFASlider3
    case 3:
        Vofa_Slider3 = DataConvert.DataFloat;
        break;
#endif
#ifdef UseVOFASlider4
    case 4:
        Vofa_Slider4 = DataConvert.DataFloat;
        break;
#endif
#ifdef UseVOFASlider5
    case 5:
        Vofa_Slider5 = DataConvert.DataFloat;
        break;
#endif
#ifdef UseVOFASlider6
    case 6:
        Vofa_Slider6 = DataConvert.DataFloat;
        break;
#endif
#ifdef UseVOFASlider7
    case 7:
        Vofa_Slider7 = DataConvert.DataFloat;
        break;
#endif
    }
}
/**
 * @brief  按钮命令处理
 * @param  命令号
 * @retval *
 */
void Vofa_Button_Handler(uint8_t Num)
{
    union VOFA_DataConvertTypeDef DataConvert;
    uint8_t i;
    for (i = 0; i < 4; i++)
    {
        DataConvert.DataChar[i] = Vofa_RX.Buff[3 + i];
    }
    switch (Num)
    {
#ifdef UseVOFAButton1
    case 1:
        Vofa_Button1 = DataConvert.DataFloat;
        break;
#endif
#ifdef UseVOFAButton2
    case 2:
        Vofa_Button2 = DataConvert.DataFloat;
        break;
#endif
#ifdef UseVOFAButton3
    case 3:
        Vofa_Button3 = DataConvert.DataFloat;
        break;
#endif
#ifdef UseVOFAButton4
    case 4:
        Vofa_Button4 = DataConvert.DataFloat;
        break;
#endif
#ifdef UseVOFAButton5
    case 5:
        Vofa_Button5 = DataConvert.DataFloat;
        break;
#endif
#ifdef UseVOFAButton6
    case 6:
        Vofa_Button6 = DataConvert.DataFloat;
        break;
#endif
#ifdef UseVOFAButton7
    case 7:
        Vofa_Button7 = DataConvert.DataFloat;
        break;
#endif
#ifdef UseVOFAButton8
    case 8:
        Vofa_Button8 = DataConvert.DataFloat;
        break;
#endif
    }
}
/**
 * @brief  按键命令处理
 * @param  命令号
 * @retval *
 */
void Vofa_Key_Handler(uint8_t Num)
{
    union VOFA_DataConvertTypeDef DataConvert;
    uint8_t i;
    for (i = 0; i < 4; i++)
    {
        DataConvert.DataChar[i] = Vofa_RX.Buff[3 + i];
    }
    switch (Num)
    {
#ifdef UseVOFAKey1
    case 1:
        Vofa_Key1 = DataConvert.DataFloat;
        break;
#endif
#ifdef UseVOFAKey2
    case 2:
        Vofa_Key2 = DataConvert.DataFloat;
        break;
#endif
#ifdef UseVOFAKey3
    case 3:
        Vofa_Key3 = DataConvert.DataFloat;
        break;
#endif
#ifdef UseVOFAKey4
    case 4:
        Vofa_Key4 = DataConvert.DataFloat;
        break;
#endif
    }
}

/**
 * @brief  摇杆命令处理
 * @param  命令号
 * @retval *
 */
void Vofa_Bar_Handler(uint8_t Num)
{
    union VOFA_DataConvertTypeDef DataConvert;
    uint8_t i;
    for (i = 0; i < 4; i++)
    {
        DataConvert.DataChar[i] = Vofa_RX.Buff[3 + i];
    }
    switch (Num)
    {
#ifdef UseVOFABar1
    case 1:
        for (i = 0; i < 4; i++)
        {
            DataConvert.DataChar[i] = Vofa_RX.Buff[3 + i];
        }
        Vofa_Bar_x1 = DataConvert.DataFloat;
        for (i = 0; i < 4; i++)
        {
            DataConvert.DataChar[i] = Vofa_RX.Buff[7 + i];
        }
        Vofa_Bar_y1 = DataConvert.DataFloat;
        break;
#endif
#ifdef UseVOFABar2
    case 2:
        for (i = 0; i < 4; i++)
        {
            DataConvert.DataChar[i] = Vofa_RX.Buff[3 + i];
        }
        Vofa_Bar_x2 = DataConvert.DataFloat;
        for (i = 0; i < 4; i++)
        {
            DataConvert.DataChar[i] = Vofa_RX.Buff[7 + i];
        }
        Vofa_Bar_y2 = DataConvert.DataFloat;
        break;
#endif
    }
}


