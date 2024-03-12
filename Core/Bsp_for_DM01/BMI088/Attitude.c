#include "Attitude.h"
#include "MahonyAHRS.h"
#include "BMI088driver.h"
#include "tim.h"
#include "cmsis_os.h"
#include "bsp_delay.h"
#include "bsp_spi.h"
#include "spi.h"
#include "vofa_setting.h"

imu_type_def imu_data;
bmi088_real_data_t bmi088_real_data;

uint8_t gyro_dma_rx_buf[SPI_DMA_GYRO_LENGHT];
uint8_t gyro_dma_tx_buf[SPI_DMA_GYRO_LENGHT] = {0x82,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
uint8_t accel_dma_rx_buf[SPI_DMA_ACCEL_LENGHT];
uint8_t accel_dma_tx_buf[SPI_DMA_ACCEL_LENGHT] = {0x92,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
uint8_t accel_temp_dma_rx_buf[SPI_DMA_ACCEL_TEMP_LENGHT];
uint8_t accel_temp_dma_tx_buf[SPI_DMA_ACCEL_TEMP_LENGHT] = {0xA2,0xFF,0xFF,0xFF};

volatile uint8_t gyro_update_flag = 0;
volatile uint8_t accel_update_flag = 0;
volatile uint8_t accel_temp_update_flag = 0;
volatile uint8_t mag_update_flag = 0;
volatile uint8_t imu_start_dma_flag = 0;
/**
  * @brief          IMU姿态解算初始化
  * @param[in]      采样率
  * @param[in]      重力加速度初始值
  * @retval         none
  */
void Attitude_Init(float sample_rate,float acc[3])
{
    Mahony_Init(sample_rate);  //设置采样率
    MahonyAHRSinit(acc[0],acc[1],acc[2],0,0,0);  //上电快速开始初始化
}
/**
  * @brief          欧拉角计算
  * @param[in]      角加速度数组
  * @param[in]      重力加速度数组
  * @retval         none
  */
void Attitude_Calculate(float gyro[3],float acc[3])
{
    Mahony_update(gyro[0],gyro[1],gyro[2],acc[0],acc[1],acc[2],0,0,0);
    Mahony_computeAngles();
    imu_data.attitude_raw[0]=getPitch();
    imu_data.attitude_raw[1]=getRoll();
    imu_data.attitude_raw[2]=getYaw();
    imu_data.error[2] += YAW_ERROR;
    imu_data.attitude_correct[0] = imu_data.attitude_raw[0];
    imu_data.attitude_correct[1] = imu_data.attitude_raw[1];
    imu_data.attitude_correct[2] = imu_data.attitude_raw[2] - imu_data.error[2];
    if(imu_data.attitude_correct[2]<-180)
        imu_data.attitude_correct[2]+=360;
    else if(imu_data.attitude_correct[2]>180)
        imu_data.attitude_correct[2]-=360;
}
/**
  * @brief          IMU恒温加热初始化
  * @retval         none
  */
void Temperature_Init()
{
    HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);                                         //恒温加热开启
    const float imu_pid[3] = {IMU_PID_K, IMU_PID_I, IMU_PID_D};                       //PID参数初始化
    PID_init(&imu_data.imu_pid, PID_DELTA, imu_pid, MPU6500_TEMP_PWM_MAX-1, 800);    //PID结构体初始化
}
/**
  * @brief          IMU恒温PID计算
  * @param[in]      温度设置/度
  * @retval         none
  */
void Temperature_Control(float t_set)
{
    PID_calc(&imu_data.imu_pid, imu_data.temperature, t_set);
    if(imu_data.imu_pid.out<0) imu_data.imu_pid.out = 0;
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, imu_data.imu_pid.out);
}
/**
* @brief Function implementing the mcuTask thread.
* @param argument: Not used
* @retval None
*/
/**
  * @brief Function 开始MCU工作任务
  * @param argument: Not used
  * @retval none
  */
void StartImuTask(void const * argument)
{
    while(BMI088_init()){}                                                                        //BMI088初始化
    BMI088_read(bmi088_real_data.gyro, bmi088_real_data.accel, &bmi088_real_data.temp);         //数据读取
    Attitude_Init(500, &bmi088_real_data.accel[3]);                                       //姿态转换初始化
    Temperature_Init();                                                                          //MCU恒温初始化

    //set spi frequency
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    if (HAL_SPI_Init(&hspi2) != HAL_OK) Error_Handler();
    SPI2_DMA_init((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);
    imu_start_dma_flag = 1;
    /* Infinite loop */
    for(;;)
    {
        //wait spi DMA tansmit done
        //等待SPI DMA传输
        if(gyro_update_flag & (1 << IMU_NOTIFY_SHFITS))
        {
            gyro_update_flag &= ~(1 << IMU_NOTIFY_SHFITS);
            BMI088_gyro_read_over(gyro_dma_rx_buf + BMI088_GYRO_RX_BUF_DATA_OFFSET, bmi088_real_data.gyro);
        }

        if(accel_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            accel_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            BMI088_accel_read_over(accel_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, bmi088_real_data.accel, &bmi088_real_data.time);
        }

        if(accel_temp_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            accel_temp_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            BMI088_temperature_read_over(accel_temp_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, &bmi088_real_data.temp);
        }

        // BMI088_read(imu_data.gyro, imu_data.accel, &imu_data.temperature);              //BMI088数据读取
        Attitude_Calculate(bmi088_real_data.gyro, bmi088_real_data.accel);           //BMI088欧拉角解算

//        Temperature_Control(CONSTANT_temperature);                                          //IMU恒温计算
        osDelay(2);
    }
    /* USER CODE END StartImuTask */
}

/**
  * @brief          根据imu_update_flag的值开启SPI DMA
  * @retval         none
  */
static void imu_cmd_spi_dma(void)
{
    //开启陀螺仪的DMA传输
    if( (gyro_update_flag & (1 << IMU_DR_SHFITS) ) && !(hspi2.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi2.hdmarx->Instance->CR & DMA_SxCR_EN)
        && !(accel_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)))
    {
        gyro_update_flag &= ~(1 << IMU_DR_SHFITS);
        gyro_update_flag |= (1 << IMU_SPI_SHFITS);

        HAL_GPIO_WritePin(SPI2_Gyro_CS_GPIO_Port, SPI2_Gyro_CS_Pin, GPIO_PIN_RESET);
        SPI2_DMA_enable((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);
        return;
    }
    //开启加速度计的DMA传输
    if((accel_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi2.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi2.hdmarx->Instance->CR & DMA_SxCR_EN)
       && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)))
    {
        accel_update_flag &= ~(1 << IMU_DR_SHFITS);
        accel_update_flag |= (1 << IMU_SPI_SHFITS);

        HAL_GPIO_WritePin(SPI2_Accel_CS_GPIO_Port, SPI2_Accel_CS_Pin, GPIO_PIN_RESET);
        SPI2_DMA_enable((uint32_t)accel_dma_tx_buf, (uint32_t)accel_dma_rx_buf, SPI_DMA_ACCEL_LENGHT);
        return;
    }

    if((accel_temp_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi2.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi2.hdmarx->Instance->CR & DMA_SxCR_EN)
       && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_update_flag & (1 << IMU_SPI_SHFITS)))
    {
        accel_temp_update_flag &= ~(1 << IMU_DR_SHFITS);
        accel_temp_update_flag |= (1 << IMU_SPI_SHFITS);

        HAL_GPIO_WritePin(SPI2_Accel_CS_GPIO_Port, SPI2_Accel_CS_Pin, GPIO_PIN_RESET);
        SPI2_DMA_enable((uint32_t)accel_temp_dma_tx_buf, (uint32_t)accel_temp_dma_rx_buf, SPI_DMA_ACCEL_TEMP_LENGHT);
        return;
    }
}

void DMA_Callback(void)
{
    if(__HAL_DMA_GET_FLAG(hspi2.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi2.hdmarx)) != RESET)
    {
        __HAL_DMA_CLEAR_FLAG(hspi2.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi2.hdmarx));

        //gyro read over
        //陀螺仪读取完毕
        if(gyro_update_flag & (1 << IMU_SPI_SHFITS))
        {
            gyro_update_flag &= ~(1 << IMU_SPI_SHFITS);
            gyro_update_flag |= (1 << IMU_UPDATE_SHFITS);
            HAL_GPIO_WritePin(SPI2_Gyro_CS_GPIO_Port, SPI2_Gyro_CS_Pin, GPIO_PIN_SET);
        }

        //accel read over
        //加速度计读取完毕
        if(accel_update_flag & (1 << IMU_SPI_SHFITS))
        {
            accel_update_flag &= ~(1 << IMU_SPI_SHFITS);
            accel_update_flag |= (1 << IMU_UPDATE_SHFITS);
            HAL_GPIO_WritePin(SPI2_Accel_CS_GPIO_Port, SPI2_Accel_CS_Pin, GPIO_PIN_SET);
        }
        //temperature read over
        //温度读取完毕
        if(accel_temp_update_flag & (1 << IMU_SPI_SHFITS))
        {
            accel_temp_update_flag &= ~(1 << IMU_SPI_SHFITS);
            accel_temp_update_flag |= (1 << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(SPI2_Accel_CS_GPIO_Port, SPI2_Accel_CS_Pin, GPIO_PIN_SET);
        }

        imu_cmd_spi_dma();
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == Gyro_ACCEL_Pin) {
        accel_update_flag |= 1 << IMU_DR_SHFITS;
        accel_temp_update_flag |= 1 << IMU_DR_SHFITS;
        if(imu_start_dma_flag)
        {
            imu_cmd_spi_dma();
        }
    } else if (GPIO_Pin == Gryo_INT_Pin) {
        gyro_update_flag |= 1 << IMU_DR_SHFITS;
        if(imu_start_dma_flag)
        {
            imu_cmd_spi_dma();
        }
    }
}