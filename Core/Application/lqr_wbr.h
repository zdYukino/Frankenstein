/**
  ****************************(C)ZDYUKINO***************************
  * @file       VMC.c/h
  * @brief      VMC算法代码
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
    /**输入参数 由传感器更新**/
    float phi1;    //前大腿与L5夹角 水平为 pi/2  rad
    float phi4;    //后大腿与L5夹角 水平为 0     rad
    float d_phi1;   //前大腿角速度    rad/s
    float d_phi4;   //后大腿角速度    rad/s
    float F0;      //2x1matrix 输入五连杆机构末端推力 N
    float Tp;      //2x1matrix 输入五连杆机构末端扭矩 N*M
    /**中间参数 由函数计算更新**/
    float YD;   //链接关节坐标
    float YB;   //链接关节坐标
    float XD;   //链接关节坐标
    float XB;   //链接关节坐标
    float LBD;  //sqrt{(XD-XB)^2+(YD-YB)^2}
    float A0;   //2*L2(XD-XB)
    float B0;   //2*L2(YD-YB)
    float C0;   //L2^2+LBD^2-L3^2
    float phi2; //链接关节角度
    float phi3; //链接关节角度
    float XC;   //直角坐标末端解算X值
    float YC;   //直角坐标末端解算Y值
    float L0;   //极坐标末端解算L值
    float phi0; //极坐标末端解算phi值

    float J[2][2];   //2x2matrix J矩阵
    /**输出参数 由上两个矩阵相乘得到**/
    float T[2];      //2x1matrix T1与T2髋关节力矩 N*M
    /**输出参数 虚拟髋点角速度**/
    float d_phi0;    //rad/s
}lqr_data_t;


#endif

