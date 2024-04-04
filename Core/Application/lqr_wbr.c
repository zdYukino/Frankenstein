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
#include "vofa_setting.h"
#include "usart.h"
#include "can.h"
#include "bsp_sbus.h"

lqr_data_t lqr_data_L;
lqr_data_t lqr_data_R;
wbr_control_data_t wbr_control_data;

uint8_t lqr_init_flag = 0;
extern uint8_t board_init_flag;

void lqr_calc(lqr_data_t *data_L, lqr_data_t *data_R, wbr_control_data_t *control_data);
static void K_matrix_calc(lqr_data_t *data, float length);
static void FN_calc(lqr_data_t *data);

/**
 * @brief  lqr数据初始化 left
 * @param  data_L：input left
 * @param  data_R：input right
 * @retval none
 */
void lqr_data_init(lqr_data_t *data_L, lqr_data_t *data_R, wbr_control_data_t *control_data)
{
    /**指针传递初始化**/
    data_L->imu_data = get_imu_measure_point();//left
    data_R->imu_data = get_imu_measure_point();//right
    data_L->wheel_motor_data = get_ddt_motor_measure_point(0);
    data_R->wheel_motor_data = get_ddt_motor_measure_point(1);
    /**PID控制初始化**/
    const float length_PID[3] = {LENGTH_P,LENGTH_I,LENGTH_D};
    PID_init(&data_L->length_pid,PID_POSITION,length_PID,100,0);   //腿长PID初始化
    PID_init(&data_R->length_pid,PID_POSITION,length_PID,100,0);   //腿长PID初始化

    const float leg_PID[3] = {LEG_P,LEG_I,LEG_D};
    PID_init(&control_data->leg_pid,PID_POSITION,leg_PID,1,0);   //双腿协调PID初始化
    PID_init(&control_data->leg_pid,PID_POSITION,leg_PID,1,0);   //双腿协调PID初始化

    const float yaw_PID[3] = {YAW_P,YAW_I,YAW_D};
    PID_init(&control_data->yaw_pid,PID_POSITION,yaw_PID,1,0);   //转向环PID初始化
    PID_init(&control_data->yaw_pid,PID_POSITION,yaw_PID,1,0);   //转向环PID初始化
    /**一阶低通滤波初始化**/
    const float length_FILTER[1] = {1};
    first_order_filter_init(&data_L->length_filter, CONTROL_LOOP_TIME, length_FILTER);
    first_order_filter_init(&data_R->length_filter, CONTROL_LOOP_TIME, length_FILTER);
    /**VMC始化**/
    vmc_init(&data_L->vmc_data, 0);//left
    vmc_init(&data_R->vmc_data, 1);//right
    /**初始值设定**/
    data_L->length_filter.out = data_L->vmc_data.L0;
    data_R->length_filter.out = data_R->vmc_data.L0;
}
/**
 * @brief  lqr传感计算数据更新
 * @param  data_L：input left
 * @param  data_R：input right
 * @retval none
 */
void lqr_data_update(lqr_data_t *data_L, lqr_data_t *data_R, wbr_control_data_t *control_data)
{
    /**传感器直出数据**/
    data_L->phi   = (-data_L->imu_data->attitude_correct[1]*(float)M_PI/180.0f);     //机体与水平倾角
    data_L->d_phi =  -imu_data.gyro_kalman[0];

    data_R->phi   = (-data_R->imu_data->attitude_correct[1]*(float)M_PI/180.0f);     //机体与水平倾角
    data_R->d_phi = -imu_data.gyro_kalman[0];

    dk_feedback_update(&data_L->vmc_data, 0);               //更新轮腿正运动学位置
    dk_feedback_update(&data_R->vmc_data, 1);               //更新轮腿正运动学位置
    /**计算后得出数据**/
    vmc_calc_reverse(&data_L->vmc_data, -data_L->vmc_data.joint_l1_data->toq, -data_L->vmc_data.joint_l4_data->toq);//VMC逆向解
    vmc_calc_reverse(&data_R->vmc_data,  data_R->vmc_data.joint_l1_data->toq,  data_R->vmc_data.joint_l4_data->toq);//VMC逆向解

    data_L->x = (-data_L->wheel_motor_data->x + data_R->wheel_motor_data->x);    //相对地位移 左右相同
    data_R->x = ( data_R->wheel_motor_data->x - data_L->wheel_motor_data->x);    //相对地位移 左右相同

    data_L->theta =    ((float)M_PI_2 - data_L->vmc_data.phi0 - data_L->phi);   //轮系与连杆倾角
    data_L->d_theta =  (- data_L->vmc_data.d_phi0-data_L->d_phi)            ;   //轮系与连杆倾角速度
    data_R->theta =    ((float)M_PI_2 - data_R->vmc_data.phi0 - data_R->phi);   //轮系与连杆倾角
    data_R->d_theta =  (- data_R->vmc_data.d_phi0-data_R->d_phi)            ;   //轮系与连杆倾角速度

    data_L->d_length[0] = (data_L->vmc_data.L0 - data_L->length_now) / CONTROL_LOOP_TIME;  //求腿长变化速度
    data_R->d_length[0] = (data_R->vmc_data.L0 - data_R->length_now) / CONTROL_LOOP_TIME;  //求腿长变化速度


    data_L->d_x =  (  (float)data_L->wheel_motor_data->int16_rpm*(float)M_PI*WHEEl_D/60.0f) + data_L->vmc_data.L0*data_L->d_theta*cosf(data_L->theta) + data_L->d_length[0]*sinf(data_L->theta);
    data_R->d_x =  (- (float)data_R->wheel_motor_data->int16_rpm*(float)M_PI*WHEEl_D/60.0f) + data_R->vmc_data.L0*data_R->d_theta*cosf(data_R->theta) + data_R->d_length[0]*sinf(data_R->theta);

    FN_calc(data_L);                                     //足端支持力解算
    FN_calc(data_R);                                     //足端支持力解算

    control_data->delta_theta = data_L->theta - data_R->theta; //双腿摆角差
    /**记录数据 下次运算使用**/
    data_L->length_now = data_L->vmc_data.L0;                 //腿长更新
    data_R->length_now = data_R->vmc_data.L0;                 //腿长更新

    data_L->d_length[1] = data_L->d_length[0];                //腿长速度记录
    data_R->d_length[1] = data_R->d_length[0];                //腿长速度记录

    data_L->d_theta_last = data_L->d_theta;                   //腿长加速度记录
    data_R->d_theta_last = data_R->d_theta;                   //腿长加速度记录
}
/**
 * @brief  lqr增益矩阵计算
 * @param  data_L：input left
 * @param  data_R：input right
 * @retval none
 */
void lqr_calc(lqr_data_t *data_L, lqr_data_t *data_R, wbr_control_data_t *control_data)
{
    lqr_data_update(data_L, data_R, control_data);                //更新机体参数

    data_L->length_set = VofaData[0];               //设置腿长参数
    data_R->length_set = VofaData[0];               //设置腿长参数

    PID_calc(&data_L->length_pid,data_L->length_now,data_L->length_set);    //腿长PID计算
    PID_calc(&data_R->length_pid,data_R->length_now,data_R->length_set);    //腿长PID计算

    PID_calc(&control_data->leg_pid,control_data->delta_theta,0);    //双腿协调PID
    PID_calc(&control_data->yaw_pid,imu_data.gyro_kalman[2],0);      //转向PID计算 逆时针为正

    data_L->vmc_data.F0 = data_L->length_pid.out+WIGHT_GAIN;                     //最终VMC支持力需求计算
    data_R->vmc_data.F0 = data_R->length_pid.out+WIGHT_GAIN;                     //最终VMC支持力需求计算


    K_matrix_calc(data_L,data_L->length_now);                                 //K矩阵计算
    K_matrix_calc(data_R,data_R->length_now);                                 //K矩阵计算
    data_L->T = data_L->K11*data_L->theta + data_L->K12*data_L->d_theta +          //LQR T计算
                data_L->K13*data_L->x     + data_L->K14*data_L->d_x     +
                data_L->K15*data_L->phi   + data_L->K16*data_L->d_phi   ;
    data_R->T = data_R->K11*data_R->theta + data_R->K12*data_R->d_theta +          //LQR T计算
                data_R->K13*data_R->x     + data_R->K14*data_R->d_x     +
                data_R->K15*data_R->phi   + data_R->K16*data_R->d_phi   ;
    data_L->Tp= data_L->K21*data_L->theta + data_L->K22*data_L->d_theta +          //LQR Tp计算
                data_L->K23*data_L->x     + data_L->K24*data_L->d_x     +
                data_L->K25*data_L->phi   + data_L->K26*data_L->d_phi   ;
    data_R->Tp= data_R->K21*data_R->theta + data_R->K22*data_R->d_theta +          //LQR Tp计算
                data_R->K23*data_R->x     + data_R->K24*data_R->d_x     +
                data_R->K25*data_R->phi   + data_R->K26*data_R->d_phi   ;

        data_L->vmc_data.Tp = (data_L->Tp - control_data->leg_pid.out);         //计算最终髋关节扭矩
        data_R->vmc_data.Tp = (data_R->Tp + control_data->leg_pid.out);         //计算最终髋关节扭矩

        data_L->T_send =  -(data_L->T + control_data->yaw_pid.out);             //计算最终驱动轮力矩
        data_R->T_send =   (data_R->T - control_data->yaw_pid.out);             //计算最终驱动轮力矩


    vmc_calc(&data_L->vmc_data);                                       //VMC计算关节力矩
    vmc_calc(&data_R->vmc_data);                                       //VMC计算关节力矩

    data_L->Tj1 = data_L->vmc_data.T[0];
    data_L->Tj2 = data_L->vmc_data.T[1];
    data_R->Tj1 = data_R->vmc_data.T[0];
    data_R->Tj2 = data_R->vmc_data.T[1];
    //确定力矩大小 位置限制 力矩限制
}
/**
 * @brief  lqr腿长对应参数计算
 * @param  data：input
 * @param  length：input
 * @retval none
 */
static void K_matrix_calc(lqr_data_t *data, float length)
{
    if(length<0.1||length>0.3) return;
    if(data->FN>=15)
    {
        data->K11 = -5.3756f;//(                                48.786f*length*length - 47.345f*length - 0.7115f); //R^2 = 0.9998
        data->K12 = -0.4237f;//(                                                      - 5.6196f*length + 0.1218f); //R^2 = 1
        data->K13 = -0.8419f;//(-55.064f*length*length*length + 42.892f*length*length - 11.463f*length - 1.1215f); //R^2 = 0.9991
        data->K14 = -1.3985f;//(-47.14f *length*length*length + 37.189f*length*length - 11.19f *length - 1.1837f); //R^2 = 0.9998
        data->K15 =  2.9261f;//(-180.55f*length*length*length + 159.05f*length*length - 53.184f*length + 8.1325f); //R^2 = 1
        data->K16 =  0.3484f;//(-8.0602f*length*length*length + 8.3599f*length*length - 3.4744f*length + 0.7793f); //R^2 = 1

        data->K21 = 3.4378f;//( 57.724f*length*length*length - 32.781f*length*length + 0.6436f*length + 3.5128f); //R^2 = 0.9998
        data->K22 = 0.3217f;//( 9.2032f*length*length*length - 7.2715f*length*length + 1.7563f*length + 0.236f ); //R^2 = 0.9969
        data->K23 = 1.0792f;//(-114.7f *length*length*length + 100.28f*length*length - 33.042f*length + 4.8043f); //R^2 = 1
        data->K24 = 1.7245f;//(-117.69f*length*length*length + 100.98f*length*length - 32.567f*length + 4.6866f); //R^2 = 0.9999
        data->K25 =10.6750f;//( 379.24f*length*length*length - 293.52f*length*length + 78.959f*length + 6.6423f); //R^2 = 0.9992
        data->K26 = 0.5656f;//( 30.996f*length*length*length - 25.034f*length*length + 7.203f *length + 0.0868f); //R^2 = 0.9997
    }
    else
    {
        data->K11 = 0;
        data->K12 = 0;
        data->K13 = 0;
        data->K14 = 0;
        data->K15 = 0;
        data->K16 = 0;

        data->K21 = ( 57.724f*length*length*length - 32.781f*length*length + 0.6436f*length + 3.5128f); //R^2 = 0.9998
        data->K22 = ( 9.2032f*length*length*length - 7.2715f*length*length + 1.7563f*length + 0.236f ); //R^2 = 0.9969
        data->K23 = 0;
        data->K24 = 0;
        data->K25 = 0;
        data->K26 = 0;
    }

 }
/**
* @brief  实时支持力结算
* @param  data：input
* @retval none
*/
static void FN_calc(lqr_data_t *data)
{
    float sin_theta = sinf(data->theta);
    float cos_theta = cosf(data->theta);
    float P = data->vmc_data.F0_reverse*cos_theta + data->vmc_data.Tp_reverse*sin_theta/data->length_now;
    float dd_Zm = (data->imu_data->accel_kalman[1]*sinf(-data->phi) + data->imu_data->accel_kalman[2]*cosf(-data->phi)) - GRAVITY;
    float dd_Zw = dd_Zm - (data->d_length[0]-data->d_length[1]) / CONTROL_LOOP_TIME * cos_theta +
                  2*data->d_length[0] * data->d_theta * sin_theta +
                  data->vmc_data.L0 * (data->d_theta-data->d_theta_last)/CONTROL_LOOP_TIME*sin_theta +
                  data->vmc_data.L0 * (data->d_theta)*(data->d_theta)*cos_theta;
    data->FN = P + WHEEl_M * GRAVITY + dd_Zw * WHEEl_M;
}
/**
  * @brief Function FREERTOS VOFA发送调试信息
  * @param argument: Not used
  * @retval none
  */
void LqrControlTask(void const * argument)
{
    VofaData[0] = 0.1f;
    for(;;)
    {
        if(lqr_init_flag == 0)
        {
            lqr_data_init(&lqr_data_L, &lqr_data_R, &wbr_control_data);
            lqr_init_flag = 1;
        }
        else
        {
            lqr_calc(&lqr_data_L, &lqr_data_R, &wbr_control_data);
        }
        osDelay(4);
    }
}
//TODO 参数拟合
//TODO 前进环
//TODO 转向环
//TODO ROLL自动平衡