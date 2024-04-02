/**
  ****************************(C)ZDYUKINO***************************
  * @file       VMC.c/h
  * @brief      VMC�㷨����
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     17-3-2024       ZDYukino        1. done
  *  https://zhuanlan.zhihu.com/p/613007726
  @verbatim
  ===================================================================
  ===================================================================
  @endverbatim
  ****************************(C)ZDYUKINO****************************
  **/
#ifndef VMC_H
#define VMC_H

#include "VMC.h"
#include "CAN_receive_dm.h"

#define L1 0.12f    //ǰ���ȳ� m
#define L2 0.20f    //ǰС�ȳ� m
#define L3 0.20f    //��С�ȳ� m
#define L4 0.12f    //����ȳ� m
#define L5 0.1016f  //�Źؽ���� m

typedef struct
{
    /**������� �ɴ���������**/
    const DM_measure_t *joint_l1_data; //�ؽڵ������ָ�봫��
    const DM_measure_t *joint_l4_data; //�ؽڵ������ָ�봫��
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

    float F0_reverse; //ʵʱ���
    float Tp_reverse; //ʵʱ���
}vmc_data_t;

extern vmc_data_t vmc_data[2];

extern void vmc_init(vmc_data_t *data, uint8_t side);
extern void vmc_calc(vmc_data_t *data);
extern void dk_feedback_update(vmc_data_t *data, uint8_t side);
extern void vmc_calc_reverse(vmc_data_t *data, float TJ1, float TJ2);
#endif

