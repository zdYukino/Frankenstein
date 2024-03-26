/**
  ****************************(C)ZDYUKINO****************************
  * @file       can_receive.c/h
  * @brief      
  *             ������CAN�жϽ��պ��������յ������,CAN���ͺ������͵���������Ƶ��.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V2.0.0     ***-**-2022     ZDYUKINO        �������canͨ�ź��� CAN1���Ƶ������ϲ���ͨ��
                                                CAN2����sbus���ݽ�����miss���ͨ��
                                                ������ͨ�ź�����δ���ã�
  *
  @verbatim
  ==============================================================================
  ==============================================================================
  @endverbatim
  ****************************(C)ZDYUKINO****************************
  */

#include "CAN_receive_dm.h"
#include "main.h"
#include "can.h"

DM_measure_t DM_Motor_measure[8];//������ݽṹ�嶨��
/**
  * @brief          floatתint ���޷�
  * @param[in]      x������ֵ
  * @param[in]      x_min:��С�޷�
  * @param[in]      x_max:����޷�
  * @param[in]      bits:λ
  * @retval         ����ֵ
  */
static int float_to_uint(float x, float x_min, float x_max, int bits){
    /// Converts a float to an unsigned int, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}
/**
  * @brief          uintתfloat ���޷�
  * @param[in]      x_int������ֵ
  * @param[in]      x_min:��С�޷�
  * @param[in]      x_max:����޷�
  * @param[in]      bits:λ
  * @retval         ����ֵ
  */
static float uint_to_float(int x_int, float x_min, float x_max, int bits){
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

/**
  * @brief          CAN1��CAN2�˲�������
  * @param[in]      CAN_number��CAN�ӿ�����
  * @retval         None
  */
void CAN_Filter_Init(uint8_t CAN_number)
{
    if(CAN_number >= 1 && CAN_number <= 2)
    {
        CAN_FilterTypeDef can_filter_st;
        can_filter_st.FilterActivation = ENABLE;
        can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
        can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
        can_filter_st.FilterIdHigh = 0x0000;
        can_filter_st.FilterIdLow = 0x0000;
        can_filter_st.FilterMaskIdHigh = 0x0000;
        can_filter_st.FilterMaskIdLow = 0x0000;
        can_filter_st.FilterBank = 0;
        can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
        can_filter_st.SlaveStartFilterBank = 14;
        HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
        HAL_CAN_Start(&hcan1);
        HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);//CAN1������
        if(CAN_number == 2)
        {
            CAN_FilterTypeDef can_filter_st2;
            can_filter_st2.FilterActivation = ENABLE;
            can_filter_st2.FilterMode = CAN_FILTERMODE_IDMASK;
            can_filter_st2.FilterScale = CAN_FILTERSCALE_32BIT;
            can_filter_st2.FilterIdHigh = 0x0000;
            can_filter_st2.FilterIdLow = 0x0000;
            can_filter_st2.FilterMaskIdHigh = 0x0000;
            can_filter_st2.FilterMaskIdLow = 0x0000;
            can_filter_st2.FilterBank = 14;
            can_filter_st2.FilterFIFOAssignment = CAN_RX_FIFO1;
            can_filter_st2.SlaveStartFilterBank =14;
            HAL_CAN_ConfigFilter(&hcan2, &can_filter_st2);
            HAL_CAN_Start(&hcan2);
            HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);//CAN2������
        }
    }
    else return;
}
/**
  * @brief          DM����ش�����ת��
  * @param[in]      ptr��������ݽṹ��ָ��
  * @param[in]      Data������ָ��
  * @retval         None
  */
static void get_dm_motor_measure(DM_measure_t *ptr, uint8_t *Data)
{
    ptr->id = (Data[0])&0x0F;
    ptr->state = (Data[0])>>4;
    ptr->int_p= ( Data[1] << 8) | Data[2];
    ptr->int_v= ( Data[3] << 4) | (Data[4] >> 4);
    ptr->int_t= ((Data[4] & 0xF) << 8) | Data[5];
    ptr->pos = uint_to_float(ptr->int_p, P_MIN, P_MAX, 16); // (-12.5,12.5)
    ptr->vel = uint_to_float(ptr->int_v, V_MIN, V_MAX, 12); // (-45.0,45.0)
    ptr->toq = uint_to_float(ptr->int_t, T_MIN, T_MAX, 12); // (-18.0,18.0)
    ptr->T_mos  = (float)(Data[6]);
    ptr->T_coil = (float)(Data[7]);
}
/**
  * @brief          hal��CAN�ص�����,���յ������
  * @param[in]      hcan:CAN���ָ��
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
    if(rx_header.StdId == MASTER_ID)
    {
        static uint8_t i;
        i = (rx_data[0]) & 0x0F;
        get_dm_motor_measure(&DM_Motor_measure[i - 1], rx_data);
    }
}
/**
  * @brief          hal��CAN�ص�����,���յ������
  * @param[in]      hcan:CAN���ָ��
  * @retval         none
  */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header, rx_data);
}
/**
  * @brief          MITЭ�����DM���
  * @param[in]      hcan    :CAN�ӿ�
  * @param[in]      id      :DM���ID
  * @param[in]      _pos    :λ������(rad)
  * @param[in]      _vel    :�ٶ�����(rad/s)
  * @param[in]      _KP     :λ��P
  * @param[in]      _KD     :λ��D
  * @param[in]      _torq   :����
  * @retval         none
  */
void MIT_motor_CTRL(CAN_HandleTypeDef *hcan,uint16_t id, float _pos, float _vel, float _KP, float _KD, float _torq)
{
    uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;  //���ݸ�ʽת��
    pos_tmp = float_to_uint(_pos, P_MIN, P_MAX, 16);
    vel_tmp = float_to_uint(_vel, V_MIN, V_MAX, 12);
    kp_tmp  = float_to_uint(_KP, KP_MIN, KP_MAX, 12);
    kd_tmp  = float_to_uint(_KD, KD_MIN, KD_MAX, 12);
    tor_tmp = float_to_uint(_torq, T_MIN, T_MAX, 12);

    CAN_TxHeaderTypeDef  DM_tx_message;        //����CAN�ṹ��
    uint8_t              DM_can_send_data[8];  //��������
    uint32_t send_mail_box;                    //NONE
    DM_tx_message.StdId = id;
    DM_tx_message.IDE = CAN_ID_STD;
    DM_tx_message.RTR = CAN_RTR_DATA;
    DM_tx_message.DLC = 0x08;
    DM_can_send_data[0] = (pos_tmp >> 8);
    DM_can_send_data[1] = pos_tmp;
    DM_can_send_data[2] = (vel_tmp >> 4);
    DM_can_send_data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
    DM_can_send_data[4] = kp_tmp;
    DM_can_send_data[5] = (kd_tmp >> 4);
    DM_can_send_data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
    DM_can_send_data[7] = tor_tmp;
    HAL_CAN_AddTxMessage(hcan, &DM_tx_message, DM_can_send_data, &send_mail_box);
}
/**
  * @brief          ���ص������ָ��
  * @param[in]      i: ������,��Χ[0,8]
  * @retval         �������ָ��
  */
const DM_measure_t *get_motor_measure_point(uint8_t i)
{
    return &DM_Motor_measure[(i&0x07)];//111B
}
/**
  * @brief          ʹ��DM���
  * @param[in]      hcan: CAN�ӿ�
  * @param[in]      id:  DM���ID��ͨ����λ�����ã�
  * @retval         none
  */
void start_motor(CAN_HandleTypeDef *hcan,uint16_t id)
{
    CAN_TxHeaderTypeDef  DM_tx_message;        //����CAN�ṹ��
    uint8_t              DM_can_send_data[8];  //��������
    uint32_t send_mail_box;                    //NONE
    DM_tx_message.StdId = id;
    DM_tx_message.IDE = CAN_ID_STD;
    DM_tx_message.RTR = CAN_RTR_DATA;
    DM_tx_message.DLC = 0x08;
    DM_can_send_data[0] = 0xFF;
    DM_can_send_data[1] = 0xFF;
    DM_can_send_data[2] = 0xFF;
    DM_can_send_data[3] = 0xFF;
    DM_can_send_data[4] = 0xFF;
    DM_can_send_data[5] = 0xFF;
    DM_can_send_data[6] = 0xFF;
    DM_can_send_data[7] = 0xFC;
    HAL_CAN_AddTxMessage(hcan, &DM_tx_message, DM_can_send_data, &send_mail_box);
}
/**
  * @brief          ʧ��DM���
  * @param[in]      hcan: CAN�ӿ�
  * @param[in]      id:  DM���ID��ͨ����λ�����ã�
  * @retval         none
  */
void lock_motor(CAN_HandleTypeDef *hcan,uint16_t id)
{
    CAN_TxHeaderTypeDef  DM_tx_message;        //����CAN�ṹ��
    uint8_t              DM_can_send_data[8];  //��������
    uint32_t send_mail_box;                    //NONE
    DM_tx_message.StdId = id;
    DM_tx_message.IDE = CAN_ID_STD;
    DM_tx_message.RTR = CAN_RTR_DATA;
    DM_tx_message.DLC = 0x08;
    DM_can_send_data[0] = 0xFF;
    DM_can_send_data[1] = 0xFF;
    DM_can_send_data[2] = 0xFF;
    DM_can_send_data[3] = 0xFF;
    DM_can_send_data[4] = 0xFF;
    DM_can_send_data[5] = 0xFF;
    DM_can_send_data[6] = 0xFF;
    DM_can_send_data[7] = 0xFD;
    HAL_CAN_AddTxMessage(hcan , &DM_tx_message, DM_can_send_data, &send_mail_box);
}
/**
  * @brief          ���õ�����
  * @param[in]      hcan: CAN�ӿ�
  * @param[in]      id:  DM���ID��ͨ����λ�����ã�
  * @retval         none
  */
void set_zero_motor(CAN_HandleTypeDef *hcan,uint16_t id)
{
    CAN_TxHeaderTypeDef  DM_tx_message;        //����CAN�ṹ��
    uint8_t              DM_can_send_data[8];  //��������
    uint32_t send_mail_box;                    //NONE
    DM_tx_message.StdId = id;
    DM_tx_message.IDE = CAN_ID_STD;
    DM_tx_message.RTR = CAN_RTR_DATA;
    DM_tx_message.DLC = 0x08;
    DM_can_send_data[0] = 0xFF;
    DM_can_send_data[1] = 0xFF;
    DM_can_send_data[2] = 0xFF;
    DM_can_send_data[3] = 0xFF;
    DM_can_send_data[4] = 0xFF;
    DM_can_send_data[5] = 0xFF;
    DM_can_send_data[6] = 0xFF;
    DM_can_send_data[7] = 0xFE;
    HAL_CAN_AddTxMessage(hcan , &DM_tx_message, DM_can_send_data, &send_mail_box);
}
