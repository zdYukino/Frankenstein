#include "Attitude.h"
#include "MahonyAHRS.h"
#include "BMI088driver.h"
#include "tim.h"
#include "cmsis_os.h"
#include "bsp_delay.h"

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
    imu_data.error[0] = 0;
    imu_data.error[1] = 0;
    imu_data.error[2] = 0;
}
/**
  * @brief          欧拉角计算
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
    if(imu_data.attitude_correct[2]<-180)
        imu_data.attitude_correct[2]+=360;
    else if(imu_data.attitude_correct[2]>180)
        imu_data.attitude_correct[2]-=360;

//    imu_data.attitude_correct[2] = imu_data.attitude_raw[2];
}
/**
  * @brief          IMU恒温加热初始化
  * @retval         none
  */
void Temperature_Init()
{
    HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);                                         //恒温加热开启
    const float imu_pid[3] = {IMU_PID_K, IMU_PID_I, IMU_PID_D};                       //PID参数初始化
    PID_init(&imu_data.imu_pid, PID_DELTA, imu_pid, MPU6500_TEMP_PWM_MAX-1, 800);    //PID结构体初始化
}
/**
  * @brief          IMU恒温PID计算
  * @param[in]      温度设置/度
  * @retval         none
  */
void Temperature_Control(float t_set)
{
    PID_calc(&imu_data.imu_pid, imu_data.temperature, t_set);
    if(imu_data.imu_pid.out<0) imu_data.imu_pid.out = 0;
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, imu_data.imu_pid.out);
}
/**
* @brief Function implementing the mcuTask thread.
* @param argument: Not used
* @retval None
*/
/**
  * @brief Function 开始MCU工作任务
  * @param argument: Not used
  * @retval none
  */
void StartImuTask(void const * argument)
{
    while(BMI088_init()){}                                                                        //BMI088初始化
    BMI088_read(imu_data.gyro, imu_data.accel, &imu_data.temperature);                  //数据读取
    Attitude_Init(500, &imu_data.accel[3]);                                       //姿态转换初始化
    Temperature_Init();                                                                          //MCU恒温初始化
    /* Infinite loop */
    for(;;)
    {
        BMI088_read(imu_data.gyro, imu_data.accel, &imu_data.temperature);              //BMI088数据读取
        Attitude_Calculate(imu_data.gyro, imu_data.accel);                                   //BMI088欧拉角解算
        Temperature_Control(CONSTANT_temperature);                                          //IMU恒温计算
        DWT_DelayUS(1000);
        osDelay(1);
    }
    /* USER CODE END StartImuTask */
}
