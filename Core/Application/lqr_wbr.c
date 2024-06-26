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
void remote_control(wbr_control_data_t *control_Data, uint16_t sbus[]);
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
    PID_init(&data_L->length_pid,PID_POSITION,length_PID,100,2);   //腿长PID初始化
    PID_init(&data_R->length_pid,PID_POSITION,length_PID,100,2);   //腿长PID初始化

    const float leg_PID[3] = {LEG_P,LEG_I,LEG_D};
    PID_init(&control_data->leg_pid,PID_POSITION,leg_PID,1,0);   //双腿协调PID初始化

    const float yaw_PID[3] = {YAW_P,YAW_I,YAW_D};
    PID_init(&control_data->yaw_pid,PID_POSITION,yaw_PID,1,0);   //转向环PID初始化

    const float roll_PID[3] = {ROLL_P,ROLL_I,ROLL_D};
    PID_init(&control_data->roll_pid,PID_POSITION,roll_PID,5.0f,0);   //转向环PID初始化
    /**一阶低通滤波初始化**/
    const float length_FILTER[1] = {0.3f};
    first_order_filter_init(&data_L->length_filter, CONTROL_LOOP_TIME, length_FILTER);
    first_order_filter_init(&data_R->length_filter, CONTROL_LOOP_TIME, length_FILTER);
    /**VMC始化**/
    vmc_init(&data_L->vmc_data, 0);//left
    vmc_init(&data_R->vmc_data, 1);//right
    /**初始值设定**/
    data_L->length_filter.out = data_L->vmc_data.L0;
    data_R->length_filter.out = data_R->vmc_data.L0;

    DDT_measure[0].pos_total = 0;
    DDT_measure[1].pos_total = 0;
    control_data->delta_x = 0;
    control_data->wbr_state = FALL_STATE;
    control_data->wbr_state_last = FALL_STATE;
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
    control_data->roll_angle = data_L->imu_data->attitude_correct[0];

    data_L->phi   = (-data_L->imu_data->attitude_correct[1]*(float)M_PI/180.0f)*0.8f;     //机体与水平倾角
    data_L->d_phi =  -imu_data.gyro_kalman[0];

    data_R->phi   = (-data_R->imu_data->attitude_correct[1]*(float)M_PI/180.0f)*0.8f;     //机体与水平倾角
    data_R->d_phi = -imu_data.gyro_kalman[0];

    dk_feedback_update(&data_L->vmc_data, 0);               //更新轮腿正运动学位置
    dk_feedback_update(&data_R->vmc_data, 1);               //更新轮腿正运动学位置
    /**计算后得出数据**/
    vmc_calc_reverse(&data_L->vmc_data, -data_L->vmc_data.joint_l1_data->toq, -data_L->vmc_data.joint_l4_data->toq);//VMC逆向解
    vmc_calc_reverse(&data_R->vmc_data,  data_R->vmc_data.joint_l1_data->toq,  data_R->vmc_data.joint_l4_data->toq);//VMC逆向解


    data_L->theta =    ((float)M_PI_2 - data_L->vmc_data.phi0 - data_L->phi);   //轮系与连杆倾角
    data_L->d_theta =  (- data_L->vmc_data.d_phi0-data_L->d_phi)            ;   //轮系与连杆倾角速度
    data_R->theta =    ((float)M_PI_2 - data_R->vmc_data.phi0 - data_R->phi);   //轮系与连杆倾角
    data_R->d_theta =  (- data_R->vmc_data.d_phi0-data_R->d_phi)            ;   //轮系与连杆倾角速度

    data_L->d_length[0] = (data_L->vmc_data.L0 - data_L->length_now) / CONTROL_LOOP_TIME;  //求腿长变化速度
    data_R->d_length[0] = (data_R->vmc_data.L0 - data_R->length_now) / CONTROL_LOOP_TIME;  //求腿长变化速度

    data_L->d_x =  (  (float)data_L->wheel_motor_data->int16_rpm*(float)M_PI*WHEEl_D/60.0f) + data_L->vmc_data.L0*data_L->d_theta*cosf(data_L->theta) + data_L->d_length[0]*sinf(data_L->theta);
    data_R->d_x =  (- (float)data_R->wheel_motor_data->int16_rpm*(float)M_PI*WHEEl_D/60.0f) + data_R->vmc_data.L0*data_R->d_theta*cosf(data_R->theta) + data_R->d_length[0]*sinf(data_R->theta);

    data_L->x = (-data_L->wheel_motor_data->x + data_R->wheel_motor_data->x)*0.4f;    //相对地位移 左右相同
    data_R->x = ( data_R->wheel_motor_data->x - data_L->wheel_motor_data->x)*0.4f;    //相对地位移 左右相同

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
    lqr_data_update(data_L, data_R, control_data);                    ////更新机体参数

    remote_control(&wbr_control_data,sbus_channel);  ////遥控WBR

    data_L->x += control_data->delta_x;                               //前后运动跟踪
    data_R->x += control_data->delta_x;                               //前后运动跟踪

    PID_calc(&control_data->roll_pid,control_data->roll_angle,control_data->roll_angel_set);

    first_order_filter_cali(&data_L->length_filter, control_data->height_set);               //设置腿长参数
    first_order_filter_cali(&data_R->length_filter, control_data->height_set);               //设置腿长参数

    data_L->length_set = data_L->length_filter.out;
    data_R->length_set = data_R->length_filter.out;

    PID_calc(&data_L->length_pid,data_L->length_now,data_L->length_set);    //腿长PID计算
    PID_calc(&data_R->length_pid,data_R->length_now,data_R->length_set);    //腿长PID计算

    PID_calc(&control_data->leg_pid,control_data->delta_theta,0);    //双腿协调PID
    PID_calc(&control_data->yaw_pid,imu_data.gyro_kalman[2],control_data->yaw_speed_set);      //转向PID计算 逆时针为正

    data_L->vmc_data.F0 = data_L->length_pid.out+WIGHT_GAIN - (control_data->roll_pid.out/*横滚叠加*/); //最终VMC支持力需求计算
    data_R->vmc_data.F0 = data_R->length_pid.out+WIGHT_GAIN + (control_data->roll_pid.out/*横滚叠加*/); //最终VMC支持力需求计算


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
    data_L->T_send      =-(data_L->T + control_data->yaw_pid.out);             //计算最终驱动轮力矩
    data_R->T_send      = (data_R->T - control_data->yaw_pid.out);             //计算最终驱动轮力矩
    vmc_calc(&data_L->vmc_data);                                       //VMC计算关节力矩
    vmc_calc(&data_R->vmc_data);                                       //VMC计算关节力矩

    if(data_L->vmc_data.T[0]<4&&data_L->vmc_data.T[0]>-4)
        data_L->Tj1 = data_L->vmc_data.T[0];
    if(data_L->vmc_data.T[1]<4&&data_L->vmc_data.T[1]>-4)
        data_L->Tj2 = data_L->vmc_data.T[1];
    if(data_R->vmc_data.T[0]<4&&data_R->vmc_data.T[0]>-4)
        data_R->Tj1 = data_R->vmc_data.T[0];
    if(data_R->vmc_data.T[1]<4&&data_R->vmc_data.T[1]>-4)
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
        data->K11 =  (                                38.255f*length*length - 33.933f*length - 3.4464f);// -6.40093977819123f;//-
        data->K12 =  (                                                      - 4.2489f*length - 0.4564f);//-0.876714614073215f;//
        data->K13 =  (-37.786f*length*length*length + 29.919f*length*length - 8.1727f*length - 1.6559f);// -2.20934993393156f;//-
        data->K14 =  (-8.8170f*length*length*length + 6.0337f*length*length - 2.1238f*length - 2.0974f);// -2.25778401636481f;//-
        data->K15 =  (-164.08f*length*length*length + 159.91f*length*length - 58.171f*length + 8.8291f);//  4.44639273770525f;// 3
        data->K16 =  (-8.4338f*length*length*length + 8.5915f*length*length - 3.4069f*length + 0.6692f);// 0.406057674473306f;//
        data->K21 =  (-26.305f*length*length*length + 47.445f*length*length - 25.738f*length + 5.5108f);//  3.37538596501877f;// 4
        data->K22 =  (                                5.5640f*length*length - 3.9674f*length + 0.9059f);// 0.568495569283904f;//
        data->K23 =  (-55.784f*length*length*length + 55.322f*length*length - 20.365f*length + 3.0359f);//  1.49584281890620f;// 1
        data->K24 =  (-86.517f*length*length*length + 76.759f*length*length - 25.079f*length + 3.3132f);//  1.49005195338826f;// 1
        data->K25 =  ( 238.14f*length*length*length - 188.14f*length*length + 51.375f*length + 7.9253f);//  11.4027963927910f;// 1
        data->K26 =  ( 13.531f*length*length*length - 11.009f*length*length + 3.1650f*length + 0.3989f);// 0.617919874664229f;//
    }
    else
    {
        data->K11 = 0;
        data->K12 = 0;
        data->K13 = 0;
        data->K14 = 0;
        data->K15 = 0;
        data->K16 = 0;
        data->K21 =  (-26.305f*length*length*length + 47.445f*length*length - 25.738f*length + 5.5108f);//  3.37538596501877f;// 4
        data->K22 =  (                                5.5640f*length*length - 3.9674f*length + 0.9059f);// 0.568495569283904f;//
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
* @brief  遥控器控制
* @param  data：input
* @retval none
*/
void remote_control(wbr_control_data_t *control_Data, uint16_t sbus[])
{
    /** 位移环 **/
    if(sbus[2]>0) control_Data->speed_set = rc_dead_band_limit(((float)sbus[2]-1000),20) * 0.001f;
    else          control_Data->speed_set = 0;
    control_Data->delta_x -= wbr_control_data.speed_set*CONTROL_LOOP_TIME;
    control_Data->delta_x -= wbr_control_data.speed_set*CONTROL_LOOP_TIME;
    /** 高度环 **/
    control_Data->height_set = ((float)sbus[9]-150)/1600*0.2f+0.1f;
    if(control_Data->height_set<=0.1) control_Data->height_set = 0.1f;
    else if(control_Data->height_set>=0.3) control_Data->height_set = 0.3f;
    /** 转向环 **/
    if(sbus[3]>0) control_Data->yaw_speed_set =-rc_dead_band_limit(((float)sbus[3]-1000),20) * 0.004f;
    /** 横滚环 **/
    control_Data->roll_angel_set = 0;
}
/**
* @brief  横滚控制
* @param  data：input
* @retval none
*/
void roll_control(lqr_data_t *data_L, lqr_data_t *data_R, wbr_control_data_t *control_data)
{

}
/**
* @brief  WBR状态解算
* @param  data：input
* @retval none
*/
void wbr_state_calc(lqr_data_t *data_L, lqr_data_t *data_R, wbr_control_data_t *control_data)
{
    if(control_data->wbr_state != control_data->wbr_state_last) return;////状态转换中，跳过状态监测
    
}
/**
  * @brief Function lqr实时计算
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

        if(board_init_flag == 1)
        {
            if(sbus_channel[7] >= 1500)
            {
                MIT_motor_CTRL(&hcan1,1, 0, 0, 0, 0, -lqr_data_L.Tj1);
                osDelay(1);
                MIT_motor_CTRL(&hcan2,3, 0, 0, 0, 0,  lqr_data_R.Tj2);
                DDT_motor_toq_CTRL(&huart2, 0x01,  lqr_data_L.T_send);
                osDelay(1);
                MIT_motor_CTRL(&hcan1,2, 0, 0, 0, 0, -lqr_data_L.Tj2);
                osDelay(1);
                MIT_motor_CTRL(&hcan2,4, 0, 0, 0, 0,  lqr_data_R.Tj1);
                DDT_motor_toq_CTRL(&huart2, 0x02,  lqr_data_R.T_send);
                osDelay(1);
            }
            else {
                MIT_motor_CTRL(&hcan1, 1, 0, 0, 0, 0, 0);
                osDelay(1);
                MIT_motor_CTRL(&hcan2, 3, 0, 0, 0, 0, 0);
                DDT_motor_toq_CTRL(&huart2, 0x01, 0);
                osDelay(1);
                MIT_motor_CTRL(&hcan1, 2, 0, 0, 0, 0, 0);
                osDelay(1);
                MIT_motor_CTRL(&hcan2, 4, 0, 0, 0, 0, 0);
                DDT_motor_toq_CTRL(&huart2, 0x02, 0);
                osDelay(1);

            }
        }
        else
        osDelay(4);
    }
}
//TODO
//ROLL自动平衡