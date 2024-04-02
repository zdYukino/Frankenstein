/**
  ****************************(C)ZDYUKINO***************************
  * @file       VMC.c/h
  * @brief      VMC算法代码
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

#define CONTROL_LOOP_TIME    0.004f  //lqr计算周期  s
#define GRAVITY              9.8f    //重力加速度

#define WIGHT_GAIN 18.0f   //机体重量/2 前馈 N
#define WHEEl_D    0.102f  //轮子直径       M
#define WHEEl_M    0.8f  //轮子质量       Kg

#define LENGTH_P    400.0f  //腿长控制PID参数
#define LENGTH_I    0.0f    //腿长控制PID参数
#define LENGTH_D    200.0f  //腿长控制PID参数

#define LEG_P    1.0f     //腿长控制PID参数
#define LEG_I    0.0f     //腿长控制PID参数
#define LEG_D    2.0f     //腿长控制PID参数

#define YAW_P    2.0f     //腿长控制PID参数
#define YAW_I    0.0f     //腿长控制PID参数
#define YAW_D    1.0f     //腿长控制PID参数
typedef struct
{
    /**LQR输入参数**/
    float speed_set;                //期望速度
    float length_set;               //期望高度
    /**LQR输入传感器参数**/
    const imu_type_def *imu_data;   //imu数据指针传递
    const DDT_measure_t *wheel_motor_data;   //驱动轮电机传参
    /**LQR需要控制的参数**/
    float theta;    //轮杆夹角 rad
    float d_theta;  //轮杆角速度 rad/s
    float x;        //水平位移 m
    float d_x;      //水平速度 m/s
    float phi;      //机体与水平夹角 rad
    float d_phi;    //机体与水平角速度 rad/s
    /**LQR 通过VMC完成实际控制**/
    vmc_data_t vmc_data;
    /**LQR 中间过程参数**/
    pid_type_def length_pid;
    first_order_filter_type_t length_filter;
    float length_now;               //当前腿长
    float d_length[2];              //当前腿长速度
    float d_theta_last;             //当前腿长速度
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
    /**LQR 输出参数**/
    float Tp;   //机体端扭矩
    float T;    //足端扭矩
    float Tj1;  //髋关节扭矩
    float Tj2;  //髋关节扭矩
    float T_send;  //足端扭矩发送至电机
    float FN;      //支持力解算
}lqr_data_t;

typedef struct
{
    pid_type_def leg_pid;
    pid_type_def yaw_pid;
    /**中间过程参数**/
    float delta_theta;
    /**期望设置参数**/
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

