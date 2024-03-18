/**
  ****************************(C)ZDYUKINO***************************
  * @file       lqr_wbr.c/h
  * @brief      ����LQR���ƴ���
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     18-3-2024       ZDYukino        1. begin
  *  https://zhuanlan.zhihu.com/p/563048952
  @verbatim
  ===================================================================
  ===================================================================
  @endverbatim
  ****************************(C)ZDYUKINO****************************
  **/
#include "main.h"
#include "lqr_wbr.h"
#include "math.h"

/**
 * @brief  lqr���ݳ�ʼ��
 * @param  vmc_data_t��input
 * @retval none
 */
void lqr_data_init(lqr_data_t *data)
{
    data->imu_data = get_imu_measure_point();
    data->joint_data = get_motor_measure_point(0);
}
/**
 * @brief  lqr���м������ݸ���
 * @param  vmc_data_t��input
 * @retval none
 */
void lqr_data_update(lqr_data_t *data)
{
    /**������ֱ������**/
    data->d_phi = -data->imu_data->gyro_kalman[1];              //������ˮƽ����ٶ�
    data->phi = -data->imu_data->attitude_correct[1];           //������ˮƽ���
//    data->x =                                                   //��Ե��ٶ�
    /**�����ó�����**/
    data->d_x = data->x * CONTROL_LOOP_TIME;                    //���λ��
    data->theta = M_PI_2 - data->vmc_data.phi0 - data->phi;     //��ϵ���������
    data->d_theta =  - data->vmc_data.d_phi0 - data->d_phi;     //��ϵ����������ٶ�
}