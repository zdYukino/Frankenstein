/**
  ****************************(C)ZDYUKINO***************************
  * @file       kalman.c/h
  * @brief      �������˲�
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     3-12-2024       ZDYukino        1. done
  *  https://blog.csdn.net/weixin_45751396/article/details/119595886
  @verbatim
  ===================================================================
  ===================================================================
  @endverbatim
  ****************************(C)ZDYUKINO****************************
  **/
#include "Kalman.h"

void Kalman_Init(Kalman_t *kfp, float lastP, float q, float r)
{
	kfp->Last_P = lastP;
	kfp->Now_P = 0;
	kfp->out = 0;
	kfp->Kg = 0;
	kfp->Q = q;
	kfp->R = r;
}
/**
 *�������˲���
 *@param 	Kalman *kfp �������ṹ�����
 *   			float input ��Ҫ�˲��Ĳ����Ĳ���ֵ�����������Ĳɼ�ֵ��
 *@return �˲���Ĳ���������ֵ��
 */
float KalmanFilter(Kalman_t *kfp,float input)
{
   //Ԥ��Э����̣�kʱ��ϵͳ����Э���� = k-1ʱ�̵�ϵͳЭ���� + ��������Э����
   kfp->Now_P = kfp->Last_P + kfp->Q;
   //���������淽�̣����������� = kʱ��ϵͳ����Э���� / ��kʱ��ϵͳ����Э���� + �۲�����Э���
   kfp->Kg = kfp->Now_P / (kfp->Now_P + kfp->R);
   //��������ֵ���̣�kʱ��״̬����������ֵ = ״̬������Ԥ��ֵ + ���������� * ������ֵ - ״̬������Ԥ��ֵ��
   kfp->out = kfp->out + kfp->Kg * (input -kfp->out);//��Ϊ��һ�ε�Ԥ��ֵ������һ�ε����ֵ
   //����Э�����: ���ε�ϵͳЭ����� kfp->LastP ����һ������׼����
   kfp->Last_P = (1-kfp->Kg) * kfp->Now_P;
   return kfp->out;
}
