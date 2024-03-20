/**
  ****************************(C)ZDYUKINO***************************
  * @file       VMC.c/h
  * @brief      VMC算法代码
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
#include "main.h"
#include "VMC.h"
#include "math.h"

vmc_data_t vmc_data[2]; //五连杆VMC数据定义

float d_phi0_calc(float phi_1, float phi_4, float d_phi1, float d_phi4);
void vmc_calc(vmc_data_t *data);
/**
 * @brief  VMC数据初始化 right
 * @param  data：input VMC结构体
 * @param  side：input 选边初始化 0左侧 1右侧
 * @retval none
 */
void vmc_init(vmc_data_t *data, uint8_t side)
{
    if(side == 0)
    {
        data->joint_l1_data = get_motor_measure_point(0);
        data->joint_l4_data = get_motor_measure_point(1);
        /**数据传递初始化**/
        data->phi1 =  -data->joint_l1_data->pos + (float)M_PI;
        data->phi4 =  -data->joint_l4_data->pos;
        data->d_phi1 = -data->joint_l1_data->vel;
        data->d_phi4 = -data->joint_l4_data->vel;
    }
    else
    {
        data->joint_l1_data = get_motor_measure_point(3);
        data->joint_l4_data = get_motor_measure_point(2);
        /**数据传递初始化**/
        data->phi1 =   data->joint_l1_data->pos + (float)M_PI;
        data->phi4 =   data->joint_l4_data->pos;
        data->d_phi1 = data->joint_l1_data->vel;
        data->d_phi4 = data->joint_l4_data->vel;
    }
    data->F0 = 0;
    data->Tp = 0;
}
/**
 * @brief  VMC数据初始化 right
 * @param  data：input VMC结构体
 * @param  side：input 选边初始化 0左侧 1右侧
 * @retval none
 */
void vmc_feedback_update(vmc_data_t *data, uint8_t side, float get_F0, float get_Tp)
{
    if(side == 0)
    {
        /**数据传递**/
        data->phi1 = -data->joint_l1_data->pos + (float)M_PI;
        data->phi4 = -data->joint_l4_data->pos;
        data->d_phi1 = -data->joint_l1_data->vel;
        data->d_phi4 = -data->joint_l4_data->vel;
    }
    else
    {
        /**数据传递**/
        data->phi1 = data->joint_l1_data->pos + (float)M_PI;
        data->phi4 = data->joint_l4_data->pos;
        data->d_phi1 = data->joint_l1_data->vel;
        data->d_phi4 = data->joint_l4_data->vel;
    }
    /**计算参数赋值**/
    data->F0 = get_F0;
    data->Tp = get_Tp;
    vmc_calc(data);
}
/**
 * @brief  VMC数据计算
 * @param  data：input VMC结构体
 * @retval none
 */
void vmc_calc(vmc_data_t *data)
{
    data->YD = L4*sinf(data->phi4);
    data->YB = L1*sinf(data->phi1);
    data->XD = L5 + L4*cosf(data->phi4);
    data->XB = L1*cosf(data->phi1);
    data->LBD = sqrtf((data->XD - data->XB)*(data->XD - data->XB) + (data->YD - data->YB)*(data->YD - data->YB));
    data->A0 = 2.0f*L2*(data->XD - data->XB);
    data->B0 = 2.0f*L2*(data->YD - data->YB);
    data->C0 = L2*L2 + data->LBD*data->LBD - L3*L3;

    data->phi2 = 2.0f*atan2f((data->B0 + sqrtf(data->A0*data->A0 + data->B0*data->B0 - data->C0*data->C0)),data->A0 + data->C0);
    data->phi3 = atan2f(data->YB-data->YD+L2*sinf(data->phi2),data->XB-data->XD+L2*cosf(data->phi2));
    data->XC = L1*cosf(data->phi1) + L2*cosf(data->phi2);
    data->YC = L1*sinf(data->phi1) + L2*sinf(data->phi2);
    data->L0 = sqrtf((data->XC - L5/2)*(data->XC - L5/2) + data->YC*data->YC);
    data->phi0 = atan2f(data->YC,data->XC - L5/2);

    data->J[0][0] = (L1*sinf(data->phi0-data->phi3)*sinf(data->phi1-data->phi2))/sinf(data->phi3-data->phi2);
    data->J[0][1] = (L1*cosf(data->phi0-data->phi3)*sinf(data->phi1-data->phi2))/data->L0*sinf(data->phi3-data->phi2);
    data->J[1][0] = (L4*sinf(data->phi0-data->phi2)*sinf(data->phi3-data->phi4))/sinf(data->phi3-data->phi2);
    data->J[1][1] = (L4*cosf(data->phi0-data->phi2)*sinf(data->phi3-data->phi4))/data->L0*sinf(data->phi3-data->phi2);

    data->T[0] = data->J[0][0]*data->F0 + data->J[0][1]*data->Tp;
    data->T[1] = data->J[1][0]*data->F0 + data->J[1][1]*data->Tp;

    data->d_phi0 = d_phi0_calc(data->phi1, data->phi4, data->d_phi1, data->d_phi4);

}

float d_phi0_calc(float phi_1, float phi_4, float d_phi1, float d_phi4)
{
    float a_tmp;
    float a_tmp_tmp;
    float b_a_tmp;
    float b_a_tmp_tmp;
    float b_d_phi0_tmp;
    float b_d_phi0_tmp_tmp;
    float b_d_phi0_tmp_tmp_tmp;
    float c_a_tmp;
    float c_a_tmp_tmp;
    float c_d_phi0_tmp;
    float c_d_phi0_tmp_tmp;
    float d_a_tmp;
    float d_d_phi0_tmp;
    float d_d_phi0_tmp_tmp;
    float d_phi0_tmp;
    float d_phi0_tmp_tmp;
    float d_phi0_tmp_tmp_tmp;
    float e_a_tmp;
    float e_d_phi0_tmp_tmp;
    float f_d_phi0_tmp_tmp;
    float g_d_phi0_tmp_tmp;
    float h_d_phi0_tmp_tmp;
    d_phi0_tmp = (float)cosf(phi_1);
    b_d_phi0_tmp = (float)sinf(phi_1);
    d_phi0_tmp_tmp_tmp = 0.12F * d_phi0_tmp;
    d_phi0_tmp_tmp = (float)cosf(phi_4);
    b_d_phi0_tmp_tmp = (0.1016F - d_phi0_tmp_tmp_tmp) + 0.12F * d_phi0_tmp_tmp;
    b_d_phi0_tmp_tmp_tmp = 0.12F * b_d_phi0_tmp;
    c_d_phi0_tmp_tmp = (float)sinf(phi_4);
    d_d_phi0_tmp_tmp = b_d_phi0_tmp_tmp_tmp - 0.12F * c_d_phi0_tmp_tmp;
    e_d_phi0_tmp_tmp = 0.24F * d_phi0_tmp * d_d_phi0_tmp_tmp;
    f_d_phi0_tmp_tmp = 0.24F * b_d_phi0_tmp * b_d_phi0_tmp_tmp;
    g_d_phi0_tmp_tmp = 0.24F * d_phi0_tmp_tmp * d_d_phi0_tmp_tmp;
    h_d_phi0_tmp_tmp = 0.24F * c_d_phi0_tmp_tmp * b_d_phi0_tmp_tmp;
    a_tmp_tmp = d_d_phi0_tmp_tmp * d_d_phi0_tmp_tmp;
    b_a_tmp_tmp = b_d_phi0_tmp_tmp * b_d_phi0_tmp_tmp;
    a_tmp = ((a_tmp_tmp + b_a_tmp_tmp) + 0.04F) - 0.04F;
    b_a_tmp = a_tmp + 0.4F * b_d_phi0_tmp_tmp;
    a_tmp_tmp =
            (float)sqrtf((0.16F * a_tmp_tmp - a_tmp * a_tmp) + 0.16F * b_a_tmp_tmp);
    c_a_tmp = a_tmp_tmp - 0.4F * d_d_phi0_tmp_tmp;
    d_a_tmp = 2.0F * (float)atan2f(c_a_tmp, b_a_tmp);
    b_a_tmp_tmp = (float)cosf(d_a_tmp);
    e_a_tmp = (0.2F * b_a_tmp_tmp - 0.0508F) + d_phi0_tmp_tmp_tmp;
    c_a_tmp_tmp = (float)sinf(d_a_tmp);
    d_a_tmp = 0.2F * c_a_tmp_tmp + b_d_phi0_tmp_tmp_tmp;
    c_d_phi0_tmp = b_a_tmp * b_a_tmp;
    a_tmp_tmp *= 2.0F;
    d_phi0_tmp =
            (((0.0384F * d_phi0_tmp * d_d_phi0_tmp_tmp -
               2.0F * (e_d_phi0_tmp_tmp + f_d_phi0_tmp_tmp) * a_tmp) +
              0.0384F * b_d_phi0_tmp * b_d_phi0_tmp_tmp) /
             a_tmp_tmp -
             0.048F * d_phi0_tmp) /
            b_a_tmp -
            c_a_tmp *
            ((0.048F * b_d_phi0_tmp + e_d_phi0_tmp_tmp) + f_d_phi0_tmp_tmp) /
            c_d_phi0_tmp;
    b_d_phi0_tmp = c_a_tmp * c_a_tmp / c_d_phi0_tmp + 1.0F;
    d_d_phi0_tmp = e_a_tmp * e_a_tmp;
    f_d_phi0_tmp_tmp = 0.4F * b_a_tmp_tmp;
    e_d_phi0_tmp_tmp = 0.4F * c_a_tmp_tmp;
    c_d_phi0_tmp =
            (((0.0384F * d_phi0_tmp_tmp * d_d_phi0_tmp_tmp -
               2.0F * (g_d_phi0_tmp_tmp + h_d_phi0_tmp_tmp) * a_tmp) +
              0.0384F * c_d_phi0_tmp_tmp * b_d_phi0_tmp_tmp) /
             a_tmp_tmp -
             0.048F * d_phi0_tmp_tmp) /
            b_a_tmp -
            c_a_tmp *
            ((0.048F * c_d_phi0_tmp_tmp + g_d_phi0_tmp_tmp) + h_d_phi0_tmp_tmp) /
            c_d_phi0_tmp;
    a_tmp_tmp = d_a_tmp * d_a_tmp / d_d_phi0_tmp + 1.0F;
    return d_phi1 *
           ((d_phi0_tmp_tmp_tmp +
             f_d_phi0_tmp_tmp * d_phi0_tmp / b_d_phi0_tmp) /
            e_a_tmp +
            d_a_tmp *
            (b_d_phi0_tmp_tmp_tmp +
             e_d_phi0_tmp_tmp * d_phi0_tmp / b_d_phi0_tmp) /
            d_d_phi0_tmp) /
           a_tmp_tmp -
           d_phi4 *
           (f_d_phi0_tmp_tmp * c_d_phi0_tmp / (b_d_phi0_tmp * e_a_tmp) +
            e_d_phi0_tmp_tmp * d_a_tmp * c_d_phi0_tmp /
            (b_d_phi0_tmp * d_d_phi0_tmp)) /
           a_tmp_tmp;
}
