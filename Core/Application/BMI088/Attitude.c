#include "main.h"
#include "Attitude.h"
#include "MahonyAHRS.h"

void Attitude_Init(uint32_t sample_rate,float acc[3])
{
    Mahony_Init(sample_rate);  //设置采样率
    MahonyAHRSinit(acc[0],acc[1],acc[2],0,0,0);  //上电快速开始初始化
}

void Attitude_Calculate(float gyro[3],float acc[3],float eulerAngle[3])
{
    Mahony_update(gyro[0],gyro[1],gyro[2],acc[0],acc[1],acc[2],0,0,0);
    Mahony_computeAngles();
    eulerAngle[0]=getPitch();
    eulerAngle[1]=getRoll();
    eulerAngle[2]=getYaw();
}
