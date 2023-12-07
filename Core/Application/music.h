/**
  ****************************(C)ZDYUKINO***************************
  * @file       music.c/h
  * @brief      ��Դ����������
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

//�������� Ĭ��F4���� 168/2 = 84Mhz
#define MAIN_FRQ 84000000
//��ʱ����װ���С
#define MAIN_RELOAD 30

void init_music(void);

#endif //MUSIC_H
