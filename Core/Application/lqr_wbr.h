/**
  ****************************(C)ZDYUKINO***************************
  * @file       VMC.c/h
  * @brief      VMC�㷨����
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     17-3-2024       ZDYukino        1. begin
  *  https://zhuanlan.zhihu.com/p/613007726
  @verbatim
  ===================================================================
  ===================================================================
  @endverbatim
  ****************************(C)ZDYUKINO****************************
  **/
#ifndef LQR_WBR_H
#define LQR_WBR_H

#include "lqr_wbr.h"
#include "VMC.h"
#include "Attitude.h"
#include "CAN_receive_dm.h"

#define CONTROL_LOOP_TIME    0.002f  //lqr��������  s

#define WIGHT_GAIN 36   //��������ǰ�� N
#define WHEEl_R    0.1  //���Ӱ뾶    M

typedef struct
{
    /**LQR�������**/
    float speed_det;                //�����ٶ�
    float length;                   //�����߶�
    /**LQR���봫��������**/
    const imu_type_def *imu_data;   //imu����ָ�봫��
    const DM_measure_t *joint_data; //�������ָ�봫��
    /**LQR��Ҫ���ƵĲ���**/
    float theta;    //�ָ˼н� rad
    float d_theta;  //�ָ˽��ٶ� rad/s
    float x;        //ˮƽλ�� m
    float d_x;      //ˮƽ�ٶ� m/s
    float phi;      //������ˮƽ�н� rad
    float d_phi;    //������ˮƽ���ٶ� rad/s
    /**LQR ͨ��VMC���ʵ�ʿ���**/
    vmc_data_t vmc_data;
    /**LQR �м���̲���**/
    pid_type_def length_pid;
    /**LQR GAIN MATRIX**/
    float K11;
    float K12;
    float K13;
    float K14;
    float K15;
    float K16;
    float K21;
    float K22;
    float K23;
    float K24;
    float K25;
    float K26;
}lqr_data_t;


#endif

