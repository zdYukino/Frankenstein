/**
  ****************************(C)ZDYUKINO***************************
  * @file       music.c/h
  * @brief      无源蜂鸣器驱动
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     7-12-2023       ZDYukino        1. done
  *
  @verbatim
  ===================================================================
  ===================================================================
  @endverbatim
  ****************************(C)ZDYUKINO****************************
  **/
#include "main.h"
#include "tim.h"
#include "music.h"
/**
  * @brief          播放音符
  * @param[in]      频率输入
  * @param[in]      延音时间
  * @param[in]      空拍时间
  * @retval         none
  */
static void music_play(uint16_t feq, uint8_t delay, uint8_t empty)
{
    if(feq != 0)//是否为空拍
    {
        uint16_t pcr = MAIN_FRQ/MAIN_RELOAD/feq;
        __HAL_TIM_SET_PRESCALER(&htim2, pcr - 1);
    }
    else
    {
        __HAL_TIM_SET_PRESCALER(&htim2, 0);
    }
    for (uint8_t i = 0; i < delay; i++)//延音
    {
        HAL_Delay(90);
    }
    __HAL_TIM_SET_PRESCALER(&htim2, 0);//空拍
    for (uint8_t i = 0; i < empty; i++)
    {
        HAL_Delay(90);
    }
}

/**
  * @brief          初始化音乐
  * @retval         none
  */
void init_music(void)
{
    HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_4, 20);
    music_play(1046, 4, 1);//11
    music_play(932, 4, 1);//#7
    music_play(1046, 1, 0);//11
    music_play(1245, 2, 0);//33
    music_play(1397, 3, 0);//44
    music_play(1480, 1, 1);//#44
    music_play(0, 1, 1);//0
    music_play(1568, 1, 1);//555
    music_play(1568, 1, 1);//555
    music_play(1568, 1, 1);//555
    music_play(1976, 1, 1);//777
    music_play(1976, 1, 1);//777
    music_play(1976, 1, 1);//777
    music_play(1976, 1, 1);//777
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_4, 0);
}