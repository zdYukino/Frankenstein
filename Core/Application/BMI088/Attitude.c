#include "main.h"
#include "Attitude.h"
#include "MahonyAHRS.h"

imu_type_def imu_data;

/**
  * @brief          IMU姿态解算初始化
  * @param[in]      采样率
  * @param[in]      重力加速度初始值
  * @retval         none
  */
void Attitude_Init(float sample_rate,float acc[3])
{
    Mahony_Init(sample_rate);  //设置采样率
    MahonyAHRSinit(acc[0],acc[1],acc[2],0,0,0);  //上电快速开始初始化
}
/**
  * @brief          UART1-6中断接收服务函数
  * @param[in]      角加速度数组
  * @param[in]      重力加速度数组
  * @retval         none
  */
void Attitude_Calculate(float gyro[3],float acc[3])
{
    Mahony_update(gyro[0],gyro[1],gyro[2],acc[0],acc[1],acc[2],0,0,0);
    Mahony_computeAngles();
    imu_data.attitude_raw[0]=getPitch();
    imu_data.attitude_raw[1]=getRoll();
    imu_data.attitude_raw[2]=getYaw();
    imu_data.error[2] += YAW_ERROR;
    imu_data.attitude_correct[0] = imu_data.attitude_raw[0];
    imu_data.attitude_correct[1] = imu_data.attitude_raw[1];
    imu_data.attitude_correct[2] = imu_data.attitude_raw[2] - imu_data.error[2];
}
