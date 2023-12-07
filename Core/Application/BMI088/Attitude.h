#ifndef TC264_ATTITUDE_H
#define TC264_ATTITUDE_H

void Attitude_Init(uint32_t sample_rate,float acc[3]);//attitude init

void Attitude_Calculate(float gyro[3],float acc[3],float eulerAngle[3]);
#endif
