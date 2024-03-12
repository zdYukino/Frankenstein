#ifndef Kalman_H
#define Kalman_H

#include "main.h"

typedef struct 
{
    float Last_P;//�ϴι���Э���� ������Ϊ0 ! ! ! ! ! 
    float Now_P;//��ǰ����Э����
    float out;//�������˲������
    float Kg;//����������
    float Q;//��������Э����
    float R;//�۲�����Э����
}Kalman_t;

extern void Kalman_Init(Kalman_t *kfp, float lastP, float q, float r);
extern float KalmanFilter(Kalman_t *kfp,float input);


#endif
