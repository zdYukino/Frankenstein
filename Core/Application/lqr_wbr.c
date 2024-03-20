/**
  ****************************(C)ZDYUKINO***************************
  * @file       lqr_wbr.c/h
  * @brief      轮腿LQR控制代码
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
#include "cmsis_os.h"

lqr_data_t lqr_data_L;
lqr_data_t lqr_data_R;
/**
 * @brief  lqr数据初始化 left
 * @param  vmc_data_t：input
 * @retval none
 */
void lqr_data_init(lqr_data_t *data_L, lqr_data_t *data_R)
{
    /**指针传递初始化**/
    data_L->imu_data = get_imu_measure_point();//left
    data_R->imu_data = get_imu_measure_point();//right
    /**VMC始化**/
    vmc_init(&data_L->vmc_data, 0);//left
    vmc_init(&data_R->vmc_data, 1);//right

}
/**
 * @brief  lqr传感计算数据更新
 * @param  vmc_data_t：input
 * @retval none
 */
void lqr_data_update(lqr_data_t *data_L, lqr_data_t *data_R)
{
/*    *//**传感器直出数据**//*
    data->d_phi = -data->imu_data->gyro_kalman[1];              //机体与水平倾角速度
    data->phi = -data->imu_data->attitude_correct[1];           //机体与水平倾角
//    data->x =                                                   //相对地速度
    *//**计算后得出数据**//*
    data->d_x = data->x * CONTROL_LOOP_TIME;                    //相对位移
    data->theta = M_PI_2 - data->vmc_data.phi0 - data->phi;     //轮系与连杆倾角
    data->d_theta =  - data->vmc_data.d_phi0 - data->d_phi;     //轮系与连杆倾角速度*/
    vmc_feedback_update(&data_L->vmc_data, 0, 0, 0);
}

void LqrControlTask(void const * argument)
{
    lqr_data_init(&lqr_data_L,&lqr_data_R);
    for(;;)
    {
        lqr_data_update(&lqr_data_L, &lqr_data_R);
        osDelay(2);
    }
}