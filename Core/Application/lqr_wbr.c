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

lqr_data_t lqr_data_L;
lqr_data_t lqr_data_R;

void lqr_calc(lqr_data_t *data_L, lqr_data_t *data_R);
void K_matrix_calc(lqr_data_t *data, float length);
/**
 * @brief  lqr���ݳ�ʼ�� left
 * @param  data_L��input left
 * @param  data_R��input right
 * @retval none
 */
void lqr_data_init(lqr_data_t *data_L, lqr_data_t *data_R)
{
    /**ָ�봫�ݳ�ʼ��**/
    data_L->imu_data = get_imu_measure_point();//left
    data_R->imu_data = get_imu_measure_point();//right
    /**PID���Ƴ�ʼ��**/
    float length_PID[3] = {LENGTH_P,LENGTH_I,LENGTH_D};
    PID_init(&data_L->length_pid,PID_POSITION,length_PID,150,0);   //�ȳ�PID��ʼ��
    PID_init(&data_R->length_pid,PID_POSITION,length_PID,150,0);   //�ȳ�PID��ʼ��
    /**VMCʼ��**/
    vmc_init(&data_L->vmc_data, 0);//left
    vmc_init(&data_R->vmc_data, 1);//right
}
/**
 * @brief  lqr���м������ݸ���
 * @param  data_L��input left
 * @param  data_R��input right
 * @retval none
 */
void lqr_data_update(lqr_data_t *data_L, lqr_data_t *data_R)
{
    /**������ֱ������**/
    data_L->d_phi = -data_L->imu_data->gyro_kalman[1];          //������ˮƽ����ٶ�
    data_L->phi   = -data_L->imu_data->attitude_correct[1]*(float)M_PI/180.0f;     //������ˮƽ���
    data_R->d_phi = -data_R->imu_data->gyro_kalman[1];          //������ˮƽ����ٶ�
    data_R->phi   = -data_R->imu_data->attitude_correct[1]*(float)M_PI/180.0f;     //������ˮƽ���
    /**�����ó�����**/
    data_L->x =     0;                                              //��Ե�λ��
    data_R->x =     0;                                              //��Ե�λ��
    data_L->d_x =   0;                                              //��Ե��ٶ�
    data_R->d_x =   0;                                              //��Ե��ٶ�

    data_L->theta = M_PI_2 - data_L->vmc_data.phi0 - data_L->phi;     //��ϵ���������
    data_L->d_theta =      - data_L->vmc_data.d_phi0-data_L->d_phi;   //��ϵ����������ٶ�
    data_R->theta = M_PI_2 - data_R->vmc_data.phi0 - data_R->phi;     //��ϵ���������
    data_R->d_theta =      - data_R->vmc_data.d_phi0-data_R->d_phi;   //��ϵ����������ٶ�

    dk_feedback_update(&data_L->vmc_data, 0);               //�����������˶�ѧλ��
    dk_feedback_update(&data_R->vmc_data, 1);               //�����������˶�ѧλ��
    data_L->length_now = data_L->vmc_data.L0;
    data_R->length_now = data_R->vmc_data.L0;
}
/**
 * @brief  lqr����������
 * @param  data_L��input left
 * @param  data_R��input right
 * @retval none
 */
void lqr_calc(lqr_data_t *data_L, lqr_data_t *data_R)
{
    lqr_data_update(data_L, data_R);                //���»������

    data_L->length_set = VofaData[0];               //�����ȳ�����
    data_R->length_set = VofaData[0];               //�����ȳ�����

    PID_calc(&data_L->length_pid,data_L->length_now,data_L->length_set);    //�ȳ�PID����
    PID_calc(&data_R->length_pid,data_R->length_now,data_R->length_set);    //�ȳ�PID����

    data_L->vmc_data.F0 = data_L->length_pid.out+WIGHT_GAIN;                        //����VMC֧�����������
    data_R->vmc_data.F0 = data_R->length_pid.out+WIGHT_GAIN;                        //����VMC֧�����������


    K_matrix_calc(data_L,data_L->length_now);                          //K������� ����VMC Tp����
    K_matrix_calc(data_R,data_R->length_now);                          //K������� ����VMC Tp����

    data_L->T = data_L->K11*data_L->theta + data_L->K12*data_L->d_theta +          //����L T����
                data_L->K13*data_L->x     + data_L->K14*data_L->d_x     +
                data_L->K15*data_L->phi   + data_L->K16*data_L->d_phi   ;
    data_R->T = data_R->K11*data_R->theta + data_R->K12*data_R->d_theta +          //����R T����
                data_R->K13*data_R->x     + data_R->K14*data_R->d_x     +
                data_R->K15*data_R->phi   + data_R->K16*data_R->d_phi   ;

    data_L->Tp= data_L->K21*data_L->theta + data_L->K22*data_L->d_theta +          //����L Tp����
                data_L->K23*data_L->x     + data_L->K24*data_L->d_x     +
                data_L->K25*data_L->phi   + data_L->K26*data_L->d_phi   ;
    data_R->Tp= data_R->K21*data_R->theta + data_R->K22*data_R->d_theta +          //����R Tp����
                data_R->K23*data_R->x     + data_R->K24*data_R->d_x     +
                data_R->K25*data_R->phi   + data_R->K26*data_R->d_phi   ;

    data_L->vmc_data.Tp = data_L->Tp;
    data_R->vmc_data.Tp = data_R->Tp;
    vmc_calc(&data_L->vmc_data);                                              //VMC����ؽ�����
    vmc_calc(&data_R->vmc_data);                                              //VMC����ؽ�����
    data_L->Tj1 = data_L->vmc_data.T[0];
    data_L->Tj2 = data_L->vmc_data.T[1];
    data_R->Tj1 = data_R->vmc_data.T[0];
    data_R->Tj2 = data_R->vmc_data.T[1];
    //ȷ�����ش�С λ������ ��������
}
/**
 * @brief  lqr�ȳ���Ӧ��������
 * @param  data��input
 * @param  length��input
 * @retval none
 */
void K_matrix_calc(lqr_data_t *data, float length)
{
    if(length<0.1||length>0.3) return;
    data->K11 =                                48.786f*length*length - 47.345f*length - 0.7115f; //R^2 = 0.9998
    data->K12 =                                                      - 5.6196f*length + 0.1218f; //R^2 = 1
    data->K13 =-55.064f*length*length*length + 42.892f*length*length - 11.463f*length - 1.1215f; //R^2 = 0.9991
    data->K14 =-47.14f *length*length*length + 37.189f*length*length - 11.19f *length - 1.1837f; //R^2 = 0.9998
    data->K15 =-180.55f*length*length*length + 159.05f*length*length - 53.184f*length + 8.1325f; //R^2 = 1
    data->K16 =-8.0602f*length*length*length + 8.3599f*length*length - 3.4744f*length + 0.7793f; //R^2 = 1
    data->K21 = 57.724f*length*length*length - 32.781f*length*length + 0.6436f*length + 3.5128f; //R^2 = 0.9998
    data->K22 = 9.2032f*length*length*length - 7.2715f*length*length + 1.7563f*length + 0.236f ; //R^2 = 0.9969
    data->K23 =-114.7f *length*length*length + 100.28f*length*length - 33.042f*length + 4.8043f; //R^2 = 1
    data->K24 =-117.69f*length*length*length + 100.98f*length*length - 32.567f*length + 4.6866f; //R^2 = 0.9999
    data->K25 = 379.24f*length*length*length - 293.52f*length*length + 78.959f*length + 6.6423f; //R^2 = 0.9992
    data->K26 = 30.996f*length*length*length - 25.034f*length*length + 7.203f *length + 0.0868f; //R^2 = 0.9997
 }
/**
  * @brief Function FREERTOS VOFA���͵�����Ϣ
  * @param argument: Not used
  * @retval none
  */
void LqrControlTask(void const * argument)
{
    VofaData[0] = 0.1f; //�ȳ�����
    lqr_data_init(&lqr_data_L,&lqr_data_R);
    for(;;)
    {
        lqr_calc(&lqr_data_L, &lqr_data_R);
        osDelay(10);
    }
}