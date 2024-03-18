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

typedef struct
{
    /**������� �ɴ���������**/
    float phi1;    //ǰ������L5�н� ˮƽΪ pi/2  rad
    float phi4;    //�������L5�н� ˮƽΪ 0     rad
    float d_phi1;   //ǰ���Ƚ��ٶ�    rad/s
    float d_phi4;   //����Ƚ��ٶ�    rad/s
    float F0;      //2x1matrix ���������˻���ĩ������ N
    float Tp;      //2x1matrix ���������˻���ĩ��Ť�� N*M
    /**�м���� �ɺ����������**/
    float YD;   //���ӹؽ�����
    float YB;   //���ӹؽ�����
    float XD;   //���ӹؽ�����
    float XB;   //���ӹؽ�����
    float LBD;  //sqrt{(XD-XB)^2+(YD-YB)^2}
    float A0;   //2*L2(XD-XB)
    float B0;   //2*L2(YD-YB)
    float C0;   //L2^2+LBD^2-L3^2
    float phi2; //���ӹؽڽǶ�
    float phi3; //���ӹؽڽǶ�
    float XC;   //ֱ������ĩ�˽���Xֵ
    float YC;   //ֱ������ĩ�˽���Yֵ
    float L0;   //������ĩ�˽���Lֵ
    float phi0; //������ĩ�˽���phiֵ

    float J[2][2];   //2x2matrix J����
    /**������� ��������������˵õ�**/
    float T[2];      //2x1matrix T1��T2�Źؽ����� N*M
    /**������� �����ŵ���ٶ�**/
    float d_phi0;    //rad/s
}lqr_data_t;


#endif

