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

lqr_data_t lqr_data_L;
lqr_data_t lqr_data_R;

uint8_t lqr_init_flag = 0;
extern uint8_t board_init_flag;
void lqr_calc(lqr_data_t *data_L, lqr_data_t *data_R);
void K_matrix_calc(lqr_data_t *data, float length);
/**
 * @brief  lqr数据初始化 left
 * @param  data_L：input left
 * @param  data_R：input right
 * @retval none
 */
void lqr_data_init(lqr_data_t *data_L, lqr_data_t *data_R)
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
    PID_init(&data_L->leg_pid,PID_POSITION,leg_PID,1,0);   //腿长PID初始化
    PID_init(&data_R->leg_pid,PID_POSITION,leg_PID,1,0);   //腿长PID初始化

    const float yaw_PID[3] = {YAW_P,YAW_I,YAW_D};
    PID_init(&data_L->yaw_pid,PID_POSITION,yaw_PID,1,0);   //腿长PID初始化
    PID_init(&data_R->yaw_pid,PID_POSITION,yaw_PID,1,0);   //腿长PID初始化
    /**VMC始化**/
    vmc_init(&data_L->vmc_data, 0);//left
    vmc_init(&data_R->vmc_data, 1);//right
}
/**
 * @brief  lqr传感计算数据更新
 * @param  data_L：input left
 * @param  data_R：input right
 * @retval none
 */
void lqr_data_update(lqr_data_t *data_L, lqr_data_t *data_R)
{
    /**传感器直出数据**/
    data_L->phi   = (-data_L->imu_data->attitude_correct[1]*(float)M_PI/180.0f);     //机体与水平倾角
    data_L->d_phi =  -imu_data.gyro_kalman[0];

    data_R->phi   = (-data_R->imu_data->attitude_correct[1]*(float)M_PI/180.0f);     //机体与水平倾角
    data_R->d_phi = -imu_data.gyro_kalman[0];

    dk_feedback_update(&data_L->vmc_data, 0);               //更新轮腿正运动学位置
    dk_feedback_update(&data_R->vmc_data, 1);               //更新轮腿正运动学位置
    /**计算后得出数据**/

    data_L->x =  -data_L->wheel_motor_data->x;
    data_R->x =   data_R->wheel_motor_data->x;                       //相对地位移

    data_L->theta =    ((float)M_PI_2 - data_L->vmc_data.phi0 - data_L->phi);   //轮系与连杆倾角
    data_L->d_theta =  (- data_L->vmc_data.d_phi0-data_L->d_phi)            ;   //轮系与连杆倾角速度
    data_R->theta =    ((float)M_PI_2 - data_R->vmc_data.phi0 - data_R->phi);   //轮系与连杆倾角
    data_R->d_theta =  (- data_R->vmc_data.d_phi0-data_R->d_phi)            ;   //轮系与连杆倾角速度

    data_L->d_x =   (float)data_L->wheel_motor_data->int16_rpm*(float)M_PI*WHEEl_D/60.0f + data_L->vmc_data.L0*data_L->d_theta*cosf(data_L->theta) + (data_L->vmc_data.L0-data_L->length_now)*sinf(data_L->theta);
    data_R->d_x =  -(float)data_R->wheel_motor_data->int16_rpm*(float)M_PI*WHEEl_D/60.0f + data_R->vmc_data.L0*data_R->d_theta*cosf(data_R->theta) + (data_R->vmc_data.L0-data_R->length_now)*sinf(data_R->theta);

    data_L->length_now = data_L->vmc_data.L0;
    data_R->length_now = data_R->vmc_data.L0;

    data_L->delta_theta = data_L->theta - data_R->theta;
}
/**
 * @brief  lqr增益矩阵计算
 * @param  data_L：input left
 * @param  data_R：input right
 * @retval none
 */
void lqr_calc(lqr_data_t *data_L, lqr_data_t *data_R)
{
    lqr_data_update(data_L, data_R);                //更新机体参数

    data_L->length_set = 0.1f;//VofaData[0];               //设置腿长参数
    data_R->length_set = 0.1f;//VofaData[0];               //设置腿长参数

    PID_calc(&data_L->length_pid,data_L->length_now,data_L->length_set);    //腿长PID计算
    PID_calc(&data_R->length_pid,data_R->length_now,data_R->length_set);    //腿长PID计算

    PID_calc(&data_L->leg_pid,data_L->delta_theta,0);    //双腿协调PID
    PID_calc(&data_L->yaw_pid,imu_data.gyro_kalman[2],0);    //转向PID计算 逆时针为正

    data_L->vmc_data.F0 = data_L->length_pid.out+WIGHT_GAIN;                        //最终VMC支持力需求计算
    data_R->vmc_data.F0 = data_R->length_pid.out+WIGHT_GAIN;                        //最终VMC支持力需求计算


    K_matrix_calc(data_L,0.1f);                          //K矩阵计算 最终VMC Tp计算
    K_matrix_calc(data_R,0.1f);                          //K矩阵计算 最终VMC Tp计算

    data_L->T = data_L->K11*data_L->theta + data_L->K12*data_L->d_theta +          //最终L T计算
                data_L->K13*data_L->x     + data_L->K14*data_L->d_x     +
                data_L->K15*data_L->phi   + data_L->K16*data_L->d_phi   ;

    data_R->T = data_R->K11*data_R->theta + data_R->K12*data_R->d_theta +          //最终R T计算
                data_R->K13*data_R->x     + data_R->K14*data_R->d_x     +
                data_R->K15*data_R->phi   + data_R->K16*data_R->d_phi   ;

    data_L->Tp= data_L->K21*data_L->theta + data_L->K22*data_L->d_theta +          //最终L Tp计算
                data_L->K23*data_L->x     + data_L->K24*data_L->d_x     +
                data_L->K25*data_L->phi   + data_L->K26*data_L->d_phi   ;

    data_R->Tp= data_R->K21*data_R->theta + data_R->K22*data_R->d_theta +          //最终R Tp计算
                data_R->K23*data_R->x     + data_R->K24*data_R->d_x     +
                data_R->K25*data_R->phi   + data_R->K26*data_R->d_phi   ;

    data_L->vmc_data.Tp = (data_L->Tp - data_L->leg_pid.out);
    data_R->vmc_data.Tp = (data_R->Tp + data_L->leg_pid.out);

    data_L->T_send =  -(data_L->T + data_L->yaw_pid.out);
    data_R->T_send =   (data_R->T - data_L->yaw_pid.out);

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
void K_matrix_calc(lqr_data_t *data, float length)
{
    if(length<0.1||length>0.3) return;
    data->K11 = (                                 48.786f*length*length - 47.345f*length - 0.7115f); //R^2 = 0.9998 -6.6220f;-3.4474f;/
    data->K12 = (                                                       - 5.6196f*length + 0.1218f); //R^2 = 1      -0.5427f;-0.3050f;/
    data->K13 = ( -55.064f*length*length*length + 42.892f*length*length - 11.463f*length - 1.1215f); //R^2 = 0.9991 -2.0980f;-1.9016f;/
    data->K14 = ( -47.14f *length*length*length + 37.189f*length*length - 11.19f *length - 1.1837f); //R^2 = 0.9998 -2.4331f;-1.7822f;/
    data->K15 = ( -180.55f*length*length*length + 159.05f*length*length - 53.184f*length + 8.1325f); //R^2 = 1       2.9590f;2.7861f;//
    data->K16 = ( -8.0602f*length*length*length + 8.3599f*length*length - 3.4744f*length + 0.7793f); //R^2 = 1       0.4039f;0.2244f;//

    data->K21 = (  57.724f*length*length*length - 32.781f*length*length + 0.6436f*length + 3.5128f); //R^2 = 0.9998 1.6033f;1.5669f;//
    data->K22 = (  9.2032f*length*length*length - 7.2715f*length*length + 1.7563f*length + 0.236f ); //R^2 = 0.9969 0.1580f;0.2069f;//
    data->K23 = ( -114.7f *length*length*length + 100.28f*length*length - 33.042f*length + 4.8043f); //R^2 = 1      1.0942f;1.6636f;//
    data->K24 = ( -117.69f*length*length*length + 100.98f*length*length - 32.567f*length + 4.6866f); //R^2 = 0.9999 1.1753f;1.4948f;//
    data->K25 = (  379.24f*length*length*length - 293.52f*length*length + 78.959f*length + 6.6423f); //R^2 = 0.9992 9.5354f;8.5053f;//
    data->K26 = (  30.996f*length*length*length - 25.034f*length*length + 7.203f *length + 0.0868f); //R^2 = 0.9997 0.5415f;0.4300f;//
 }
/**
  * @brief Function FREERTOS VOFA发送调试信息
  * @param argument: Not used
  * @retval none
  */
void LqrControlTask(void const * argument)
{
    for(;;)
    {
        if(lqr_init_flag == 0)
        {
            lqr_data_init(&lqr_data_L,&lqr_data_R);
            lqr_init_flag = 1;
        }
        else
        {
            lqr_calc(&lqr_data_L, &lqr_data_R);
        }
        if(board_init_flag == 1)
        {
            if(VofaData[1] == 1)
            {
                MIT_motor_CTRL(&hcan1,1, 0, 0, 0, 0, -lqr_data_L.Tj1);//lqr_data_L.Tj1
                osDelay(1);
                MIT_motor_CTRL(&hcan1,2, 0, 0, 0, 0, -lqr_data_L.Tj2);//lqr_data_L.Tj2
                osDelay(1);
                DDT_motor_toq_CTRL(&huart2, 0x01,  lqr_data_L.T_send);
                MIT_motor_CTRL(&hcan1,3, 0, 0, 0, 0,  lqr_data_R.Tj2);
                osDelay(1);
                MIT_motor_CTRL(&hcan1,4, 0, 0, 0, 0,  lqr_data_R.Tj1);
                osDelay(1);
                DDT_motor_toq_CTRL(&huart2, 0x02,  lqr_data_R.T_send);

//                  DDT_motor_toq_CTRL(&huart2, 0x02,  VofaData[0]);
//                  osDelay(4);
            }
            else {
                MIT_motor_CTRL(&hcan1, 1, 0, 0, 0, 0, 0);
                osDelay(1);
                MIT_motor_CTRL(&hcan1, 2, 0, 0, 0, 0, 0);
                osDelay(1);
                DDT_motor_toq_CTRL(&huart2, 0x01, 0);
                MIT_motor_CTRL(&hcan1, 3, 0, 0, 0, 0, 0);
                osDelay(1);
                MIT_motor_CTRL(&hcan1, 4, 0, 0, 0, 0, 0);
                osDelay(1);
                DDT_motor_toq_CTRL(&huart2, 0x02, 0);
//                  DDT_motor_toq_CTRL(&huart2, 0x02,  0);
//                  osDelay(4);
            }
        }
        else
        {
            osDelay(4);
        }
    }
}