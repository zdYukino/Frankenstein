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
#include "cmsis_os.h"
#include "vofa_setting.h"
#include "usart.h"
#include "can.h"
#include "bsp_sbus.h"
#include "bsp_delay.h"

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
 * @brief  lqr���ݳ�ʼ�� left
 * @param  data_L��input left
 * @param  data_R��input right
 * @retval none
 */
void lqr_data_init(lqr_data_t *data_L, lqr_data_t *data_R, wbr_control_data_t *control_data)
{
    /**ָ�봫�ݳ�ʼ��**/
    data_L->imu_data = get_imu_measure_point();//left
    data_R->imu_data = get_imu_measure_point();//right
    data_L->wheel_motor_data = get_ddt_motor_measure_point(0);
    data_R->wheel_motor_data = get_ddt_motor_measure_point(1);
    /**PID���Ƴ�ʼ��**/
    const float length_PID[3] = {LENGTH_P,LENGTH_I,LENGTH_D};
    PID_init(&data_L->length_pid,PID_POSITION,length_PID,100,2);   //�ȳ�PID��ʼ��
    PID_init(&data_R->length_pid,PID_POSITION,length_PID,100,2);   //�ȳ�PID��ʼ��

    const float leg_PID[3] = {LEG_P,LEG_I,LEG_D};
    PID_init(&control_data->leg_pid,PID_POSITION,leg_PID,1,0);   //˫��Э��PID��ʼ��
    PID_init(&control_data->leg_pid,PID_POSITION,leg_PID,1,0);   //˫��Э��PID��ʼ��

    const float yaw_PID[3] = {YAW_P,YAW_I,YAW_D};
    PID_init(&control_data->yaw_pid,PID_POSITION,yaw_PID,1,0);   //ת��PID��ʼ��
    PID_init(&control_data->yaw_pid,PID_POSITION,yaw_PID,1,0);   //ת��PID��ʼ��

    const float roll_PID[3] = {ROLL_P,ROLL_I,ROLL_D};
    PID_init(&control_data->roll_pid,PID_POSITION,roll_PID,0.2f,0);   //ת��PID��ʼ��
    PID_init(&control_data->roll_pid,PID_POSITION,roll_PID,0.2f,0);   //ת��PID��ʼ��
    /**һ�׵�ͨ�˲���ʼ��**/
    const float length_FILTER[1] = {0.3f};
    first_order_filter_init(&data_L->length_filter, CONTROL_LOOP_TIME, length_FILTER);
    first_order_filter_init(&data_R->length_filter, CONTROL_LOOP_TIME, length_FILTER);
    /**VMCʼ��**/
    vmc_init(&data_L->vmc_data, 0);//left
    vmc_init(&data_R->vmc_data, 1);//right
    /**��ʼֵ�趨**/
    data_L->length_filter.out = data_L->vmc_data.L0;
    data_R->length_filter.out = data_R->vmc_data.L0;

    DDT_measure[0].pos_total = 0;
    DDT_measure[1].pos_total = 0;
    control_data->delta_x = 0;
    control_data->wbr_state = FALL_STATE;
    control_data->wbr_state_last = FALL_STATE;
}
/**
 * @brief  lqr���м������ݸ���
 * @param  data_L��input left
 * @param  data_R��input right
 * @retval none
 */
void lqr_data_update(lqr_data_t *data_L, lqr_data_t *data_R, wbr_control_data_t *control_data)
{
    /**������ֱ������**/
    control_data->roll_angle = data_L->imu_data->attitude_correct[0];

    data_L->phi   = (-data_L->imu_data->attitude_correct[1]*(float)M_PI/180.0f);     //������ˮƽ���
    data_L->d_phi =  -imu_data.gyro_kalman[0];

    data_R->phi   = (-data_R->imu_data->attitude_correct[1]*(float)M_PI/180.0f);     //������ˮƽ���
    data_R->d_phi = -imu_data.gyro_kalman[0];

    dk_feedback_update(&data_L->vmc_data, 0);               //�����������˶�ѧλ��
    dk_feedback_update(&data_R->vmc_data, 1);               //�����������˶�ѧλ��
    /**�����ó�����**/
    vmc_calc_reverse(&data_L->vmc_data, -data_L->vmc_data.joint_l1_data->toq, -data_L->vmc_data.joint_l4_data->toq);//VMC�����
    vmc_calc_reverse(&data_R->vmc_data,  data_R->vmc_data.joint_l1_data->toq,  data_R->vmc_data.joint_l4_data->toq);//VMC�����


    data_L->theta =    ((float)M_PI_2 - data_L->vmc_data.phi0 - data_L->phi);   //��ϵ���������
    data_L->d_theta =  (- data_L->vmc_data.d_phi0-data_L->d_phi)            ;   //��ϵ����������ٶ�
    data_R->theta =    ((float)M_PI_2 - data_R->vmc_data.phi0 - data_R->phi);   //��ϵ���������
    data_R->d_theta =  (- data_R->vmc_data.d_phi0-data_R->d_phi)            ;   //��ϵ����������ٶ�

    data_L->d_length[0] = (data_L->vmc_data.L0 - data_L->length_now) / CONTROL_LOOP_TIME;  //���ȳ��仯�ٶ�
    data_R->d_length[0] = (data_R->vmc_data.L0 - data_R->length_now) / CONTROL_LOOP_TIME;  //���ȳ��仯�ٶ�

    data_L->d_x =  (  (float)data_L->wheel_motor_data->int16_rpm*(float)M_PI*WHEEl_D/60.0f) + data_L->vmc_data.L0*data_L->d_theta*cosf(data_L->theta) + data_L->d_length[0]*sinf(data_L->theta);
    data_R->d_x =  (- (float)data_R->wheel_motor_data->int16_rpm*(float)M_PI*WHEEl_D/60.0f) + data_R->vmc_data.L0*data_R->d_theta*cosf(data_R->theta) + data_R->d_length[0]*sinf(data_R->theta);

    data_L->x = (-data_L->wheel_motor_data->x + data_R->wheel_motor_data->x)*0.4f;    //��Ե�λ�� ������ͬ
    data_R->x = ( data_R->wheel_motor_data->x - data_L->wheel_motor_data->x)*0.4f;    //��Ե�λ�� ������ͬ

    FN_calc(data_L);                                     //���֧��������
    FN_calc(data_R);                                     //���֧��������

    control_data->delta_theta = data_L->theta - data_R->theta; //˫�Ȱڽǲ�
    /**��¼���� �´�����ʹ��**/
    data_L->length_now = data_L->vmc_data.L0;                 //�ȳ�����
    data_R->length_now = data_R->vmc_data.L0;                 //�ȳ�����

    data_L->d_length[1] = data_L->d_length[0];                //�ȳ��ٶȼ�¼
    data_R->d_length[1] = data_R->d_length[0];                //�ȳ��ٶȼ�¼

    data_L->d_theta_last = data_L->d_theta;                   //�ȳ����ٶȼ�¼
    data_R->d_theta_last = data_R->d_theta;                   //�ȳ����ٶȼ�¼
}
/**
 * @brief  lqr����������
 * @param  data_L��input left
 * @param  data_R��input right
 * @retval none
 */
void lqr_calc(lqr_data_t *data_L, lqr_data_t *data_R, wbr_control_data_t *control_data)
{
    lqr_data_update(data_L, data_R, control_data);                    ////���»������

    remote_control(&wbr_control_data,sbus_channel);  ////ң��WBR

    data_L->x += control_data->delta_x;                               //ǰ���˶�����
    data_R->x += control_data->delta_x;                               //ǰ���˶�����

    PID_calc(&control_data->roll_pid,control_data->roll_angle,control_data->roll_angel_set);

    first_order_filter_cali(&data_L->length_filter, control_data->height_set);               //�����ȳ�����
    first_order_filter_cali(&data_R->length_filter, control_data->height_set);               //�����ȳ�����

    data_L->length_set = data_L->length_filter.out - control_data->roll_pid.out;
    data_R->length_set = data_R->length_filter.out + control_data->roll_pid.out;

    PID_calc(&data_L->length_pid,data_L->length_now,data_L->length_set);    //�ȳ�PID����
    PID_calc(&data_R->length_pid,data_R->length_now,data_R->length_set);    //�ȳ�PID����

    PID_calc(&control_data->leg_pid,control_data->delta_theta,0);    //˫��Э��PID
    PID_calc(&control_data->yaw_pid,imu_data.gyro_kalman[2],control_data->yaw_speed_set);      //ת��PID���� ��ʱ��Ϊ��

    data_L->vmc_data.F0 = data_L->length_pid.out+WIGHT_GAIN;                     //����VMC֧�����������
    data_R->vmc_data.F0 = data_R->length_pid.out+WIGHT_GAIN;                     //����VMC֧�����������


    K_matrix_calc(data_L,data_L->length_now);                                 //K�������
    K_matrix_calc(data_R,data_R->length_now);                                 //K�������
    data_L->T = data_L->K11*data_L->theta + data_L->K12*data_L->d_theta +          //LQR T����
                data_L->K13*data_L->x     + data_L->K14*data_L->d_x     +
                data_L->K15*data_L->phi   + data_L->K16*data_L->d_phi   ;
    data_R->T = data_R->K11*data_R->theta + data_R->K12*data_R->d_theta +          //LQR T����
                data_R->K13*data_R->x     + data_R->K14*data_R->d_x     +
                data_R->K15*data_R->phi   + data_R->K16*data_R->d_phi   ;
    data_L->Tp= data_L->K21*data_L->theta + data_L->K22*data_L->d_theta +          //LQR Tp����
                data_L->K23*data_L->x     + data_L->K24*data_L->d_x     +
                data_L->K25*data_L->phi   + data_L->K26*data_L->d_phi   ;
    data_R->Tp= data_R->K21*data_R->theta + data_R->K22*data_R->d_theta +          //LQR Tp����
                data_R->K23*data_R->x     + data_R->K24*data_R->d_x     +
                data_R->K25*data_R->phi   + data_R->K26*data_R->d_phi   ;

    data_L->vmc_data.Tp = (data_L->Tp - control_data->leg_pid.out);         //���������Źؽ�Ť��
    data_R->vmc_data.Tp = (data_R->Tp + control_data->leg_pid.out);         //���������Źؽ�Ť��
    data_L->T_send      =-(data_L->T + control_data->yaw_pid.out);             //������������������
    data_R->T_send      = (data_R->T - control_data->yaw_pid.out);             //������������������
    vmc_calc(&data_L->vmc_data);                                       //VMC����ؽ�����
    vmc_calc(&data_R->vmc_data);                                       //VMC����ؽ�����

    if(data_L->vmc_data.T[0]<4&&data_L->vmc_data.T[0]>-4)
        data_L->Tj1 = data_L->vmc_data.T[0];
    if(data_L->vmc_data.T[1]<4&&data_L->vmc_data.T[1]>-4)
        data_L->Tj2 = data_L->vmc_data.T[1];
    if(data_R->vmc_data.T[0]<4&&data_R->vmc_data.T[0]>-4)
        data_R->Tj1 = data_R->vmc_data.T[0];
    if(data_R->vmc_data.T[1]<4&&data_R->vmc_data.T[1]>-4)
        data_R->Tj2 = data_R->vmc_data.T[1];
    //ȷ�����ش�С λ������ ��������
}
/**
 * @brief  lqr�ȳ���Ӧ��������
 * @param  data��input
 * @param  length��input
 * @retval none
 */
static void K_matrix_calc(lqr_data_t *data, float length)
{
    if(length<0.1||length>0.3) return;
    if(data->FN>=15)
    {
        data->K11 = (                                53.556f*length*length - 47.980f*length - 1.1697f); //R^2 = 0.9998  [350 1 100 50 4000 1]
        data->K12 = (                                                      - 5.4229f*length + 0.1148f); //R^2 = 0.9999
        data->K13 = (-31.422f*length*length*length + 23.651f*length*length - 6.0912f*length - 0.4418f); //R^2 = 0.9983
        data->K14 = (-38.838f*length*length*length + 29.266f*length*length - 7.9398f*length - 0.8633f); //R^2 = 0.9991
        data->K15 = (-220.48f*length*length*length + 183.60f*length*length - 55.846f*length + 6.8803f); //R^2 = 0.9999
        data->K16 = (-11.691f*length*length*length + 10.948f*length*length - 3.9904f*length + 0.6494f); //R^2 = 1
        data->K21 = (-36.344f*length*length*length + 43.495f*length*length - 21.317f*length + 5.1736f); //R^2 = 1
        data->K22 = ( 7.1820f*length*length*length - 5.4533f*length*length + 1.1580f*length + 0.2539f); //R^2 = 0.9986
        data->K23 = (-70.012f*length*length*length + 58.724f*length*length - 18.142f*length + 2.3715f); //R^2 = 0.9999
        data->K24 = (-129.69f*length*length*length + 105.90f*length*length - 31.528f*length + 3.9371f); //R^2 = 0.9998
        data->K25 = ( 355.69f*length*length*length - 266.69f*length*length + 68.177f*length + 6.2127f); //R^2 = 0.998
        data->K26 = ( 27.440f*length*length*length - 21.278f*length*length + 5.7068f*length + 0.1829f); //R^2 = 0.9992
    }
    else
    {
        data->K11 = 0;
        data->K12 = 0;
        data->K13 = 0;
        data->K14 = 0;
        data->K15 = 0;
        data->K16 = 0;
        data->K21 = (-36.344f*length*length*length + 43.495f*length*length - 21.317f*length + 5.1736f); //R^2 = 1
        data->K22 = ( 7.1820f*length*length*length - 5.4533f*length*length + 1.1580f*length + 0.2539f ); //R^2 = 0.9986
        data->K23 = 0;
        data->K24 = 0;
        data->K25 = 0;
        data->K26 = 0;
    }

 }
/**
* @brief  ʵʱ֧��������
* @param  data��input
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
* @brief  ң��������
* @param  data��input
* @retval none
*/
void remote_control(wbr_control_data_t *control_Data, uint16_t sbus[])
{
    /** λ�ƻ� **/
    if(sbus[2]>0) control_Data->speed_set = rc_dead_band_limit(((float)sbus[2]-1000),20) * 0.001f;
    else          control_Data->speed_set = 0;
    control_Data->delta_x -= wbr_control_data.speed_set*CONTROL_LOOP_TIME;
    control_Data->delta_x -= wbr_control_data.speed_set*CONTROL_LOOP_TIME;
    /** �߶Ȼ� **/
    control_Data->height_set = ((float)sbus[9]-150)/1600*0.2f+0.1f;
    if(control_Data->height_set<=0.1) control_Data->height_set = 0.1f;
    else if(control_Data->height_set>=0.3) control_Data->height_set = 0.3f;
    /** ת�� **/
    if(sbus[3]>0) control_Data->yaw_speed_set =-rc_dead_band_limit(((float)sbus[3]-1000),20) * 0.004f;
    /** ����� **/
    control_Data->roll_angel_set = 0;
}
/**
* @brief  �������
* @param  data��input
* @retval none
*/
void roll_control(lqr_data_t *data_L, lqr_data_t *data_R, wbr_control_data_t *control_data)
{

}
/**
* @brief  WBR״̬����
* @param  data��input
* @retval none
*/
void wbr_state_calc(lqr_data_t *data_L, lqr_data_t *data_R, wbr_control_data_t *control_data)
{
    if(control_data->wbr_state != control_data->wbr_state_last) return;////״̬ת���У�����״̬���
    
}
/**
  * @brief Function lqrʵʱ����
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
//ROLL�Զ�ƽ��