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
#include "ddt_m6_control.h"
#include "user_lib.h"

#define CONTROL_LOOP_TIME    0.004f  //lqr��������  s
#define GRAVITY              9.8f    //�������ٶ�

#define WIGHT_GAIN 18.0f   //��������/2 ǰ�� N
#define WHEEl_D    0.102f  //����ֱ��       M
#define WHEEl_M    0.8f  //��������       Kg

#define LENGTH_P    400.0f  //�ȳ�����PID����
#define LENGTH_I    0.0f    //�ȳ�����PID����
#define LENGTH_D    200.0f  //�ȳ�����PID����

#define LEG_P    1.0f     //�ȳ�����PID����
#define LEG_I    0.0f     //�ȳ�����PID����
#define LEG_D    2.0f     //�ȳ�����PID����

#define YAW_P    2.0f     //�ȳ�����PID����
#define YAW_I    0.0f     //�ȳ�����PID����
#define YAW_D    1.0f     //�ȳ�����PID����
typedef struct
{
    /**LQR�������**/
    float speed_set;                //�����ٶ�
    float length_set;               //�����߶�
    /**LQR���봫��������**/
    const imu_type_def *imu_data;   //imu����ָ�봫��
    const DDT_measure_t *wheel_motor_data;   //�����ֵ������
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
    first_order_filter_type_t length_filter;
    float length_now;               //��ǰ�ȳ�
    float d_length[2];              //��ǰ�ȳ��ٶ�
    float d_theta_last;             //��ǰ�ȳ��ٶ�
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
    /**LQR �������**/
    float Tp;   //�����Ť��
    float T;    //���Ť��
    float Tj1;  //�Źؽ�Ť��
    float Tj2;  //�Źؽ�Ť��
    float T_send;  //���Ť�ط��������
    float FN;      //֧��������
}lqr_data_t;

typedef struct
{
    pid_type_def leg_pid;
    pid_type_def yaw_pid;
    /**�м���̲���**/
    float delta_theta;
    /**�������ò���**/
    float speed_set;
    float yaw_speed_set;
    float height_set;
    float roll_angel_set;
}wbr_control_data_t;


extern lqr_data_t lqr_data_L;
extern lqr_data_t lqr_data_R;
extern wbr_control_data_t wbr_control_data;

extern void lqr_data_init(lqr_data_t *data_L, lqr_data_t *data_R, wbr_control_data_t *control_data);

#endif

