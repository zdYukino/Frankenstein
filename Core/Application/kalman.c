/**
  ****************************(C)ZDYUKINO***************************
  * @file       kalman.c/h
  * @brief      卡尔曼滤波
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
 *卡尔曼滤波器
 *@param 	Kalman *kfp 卡尔曼结构体参数
 *   			float input 需要滤波的参数的测量值（即传感器的采集值）
 *@return 滤波后的参数（最优值）
 */
float KalmanFilter(Kalman_t *kfp,float input)
{
   //预测协方差方程：k时刻系统估算协方差 = k-1时刻的系统协方差 + 过程噪声协方差
   kfp->Now_P = kfp->Last_P + kfp->Q;
   //卡尔曼增益方程：卡尔曼增益 = k时刻系统估算协方差 / （k时刻系统估算协方差 + 观测噪声协方差）
   kfp->Kg = kfp->Now_P / (kfp->Now_P + kfp->R);
   //更新最优值方程：k时刻状态变量的最优值 = 状态变量的预测值 + 卡尔曼增益 * （测量值 - 状态变量的预测值）
   kfp->out = kfp->out + kfp->Kg * (input -kfp->out);//因为这一次的预测值就是上一次的输出值
   //更新协方差方程: 本次的系统协方差付给 kfp->LastP 威下一次运算准备。
   kfp->Last_P = (1-kfp->Kg) * kfp->Now_P;
   return kfp->out;
}
