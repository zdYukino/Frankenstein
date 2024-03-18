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

#define CONTROL_LOOP_TIME    0.002f  //lqr计算周期  s

#define WIGHT_GAIN 36   //机体重量前馈 N
#define WHEEl_R    0.1  //轮子半径    M

typedef struct
{
    /**LQR输入参数**/
    float speed_det;                //期望速度
    float length;                   //期望高度
    /**LQR输入传感器参数**/
    const imu_type_def *imu_data;   //imu数据指针传递
    const DM_measure_t *joint_data; //点击数据指针传递
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

