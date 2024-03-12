#ifndef TC264_ATTITUDE_H
#define TC264_ATTITUDE_H

#include "pid.h"
#include "BMI088driver.h"
#include "kalman.h"

#define MPU6500_TEMP_PWM_MAX 5000
#define CONSTANT_temperature 40
#define IMU_PID_K 0.2
#define IMU_PID_I 0.01
#define IMU_PID_D 0
#define YAW_ERROR 0.00005568776666666667f

typedef struct
{
    //陀螺仪直出数据
    float gyro[3];  //陀螺仪角速度
    float accel[3]; //重力加速度
    //卡尔曼滤波后数据
    float gyro_kalman[3];  //陀螺仪角速度
    float accel_kalman[3]; //重力加速度
    float temperature;//温度
    //计算后数据
    float attitude_raw[3];      //姿态转换原始数据;
    float attitude_correct[3];  //修正后姿态数据
    float error[3];             //误差
    //PID恒温结构体
    pid_type_def imu_pid;
    //卡尔曼滤波结构体
    Kalman_t kalman_gyro[3];
    Kalman_t kalman_accel[3];
} imu_type_def;

extern imu_type_def imu_data;
extern bmi088_real_data_t bmi088_real_data;

void Attitude_Init(float sample_rate,float acc[3]);//attitude init
void Attitude_Calculate(float gyro[3],float acc[3]);
extern void DMA_Callback(void);
#endif
