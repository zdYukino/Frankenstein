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
#ifndef MUSIC_H
#define MUSIC_H

#include "main.h"
#include "tim.h"

//参数输入 默认F4参数 168/2 = 84Mhz
#define MAIN_FRQ 84000000
//定时器重装填大小
#define MAIN_RELOAD 30

void init_music(void);

#endif //MUSIC_H
