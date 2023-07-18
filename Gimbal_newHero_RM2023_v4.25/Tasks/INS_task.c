#include "INS_Task.h"

#include "main.h"

#include "cmsis_os.h"

#include "bsp_pwm.h"
#include "bsp_spi.h"
#include "bmi088driver.h"
#include "ist8310driver.h"
#include "pid.h"
#include "ahrs.h"


/*												INS_task.c
��Ҫ����	 ͨ������SPI��DMA�����ȡBMI088���������ݣ�
						 ֱ����CPU��ȡist8310���������ݣ�
						 �ô��������ݽ�����̬����				
																											��������							�����xxx��SPI��DMA����		����xxx��SPI��DMA����			��־�����½����ⲿ�ж�				
				��������λ��							�������н׶�		xxx_update_flag��BIT[3]		xxx_update_flag��BIT[2]		xxx_update_flag��BIT[1]		xxx_update_flag��BIT[0]
					��ʼ�� 																						0													0													0													0
															SPI���յ�����������					 
				�����ⲿ�ж�																		   	0													0													0													1
																����SPI��DMA����				    0													0													1													0
																������������(DMA������)
																���SPI��DMA����												
			����DMA��������ж�																		0													1													0													0
														(����������Ǵ������)
															�����ⲿ�ж�Line_0						1													0													0													0
				�����ⲿ�ж�
															����while(1)������
				����while(1)
															ת�����ݣ���̬����
*/






/**
  * @brief          ��ת������,���ٶȼƺʹ�����,��������Ư,��Ϊ�豸�в�ͬ��װ��ʽ
  * @param[out]     gyro: ������Ư����ת
  * @param[out]     accel: ������Ư����ת
  * @param[out]     mag: ������Ư����ת
  * @param[in]      bmi088: �����Ǻͼ��ٶȼ�����
  * @param[in]      ist8310: ����������
  * @retval         none
  */
static void imu_cali_slove(fp32 gyro[3], fp32 accel[3], fp32 mag[3], bmi088_real_data_t *bmi088, ist8310_real_data_t *ist8310);

/**
  * @brief          ����bmi088���¶�
  * @param[in]      temp:bmi088���¶�
  * @retval         none
  */
static void imu_temp_control(fp32 temp);

/**
  * @brief          ����imu_update_flag��ֵ����SPI DMA
  * @param[in]      temp:bmi088���¶�
  * @retval         none
  */
static void imu_cmd_spi_dma(void);

/**
  * @brief          ����IMU��Ư
  * @param[in]      none
  * @retval         none
  */
void mpu_offset_clc(void);



	/*(1)���������ݴ洢�ṹ�����*/
bmi088_real_data_t bmi088_real_data;
bmi088_real_data_t bmi088_offset_data;//�������
ist8310_real_data_t ist8310_real_data;
	/*(2)�¶�pid��ʼ�����¶ȿ��Ʋ���*/
static uint8_t first_temperate;
static const fp32 imu_temp_PID[3] = {TEMPERATURE_PID_KP, TEMPERATURE_PID_KI, TEMPERATURE_PID_KD};
static pid_type_def imu_temp_pid;
#define IMU_temp_PWM(pwm)  imu_pwm_set(pwm)  
	/*(3)������*/
static TaskHandle_t INS_Task_local_handler;
	/*(4)SPI�ľ��*/
extern SPI_HandleTypeDef hspi1;
	/*(5)DMA���պͷ��ͻ���*/
uint8_t gyro_dma_rx_buf[SPI_DMA_GYRO_LENGHT];
uint8_t gyro_dma_tx_buf[SPI_DMA_GYRO_LENGHT] = {0x82,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
uint8_t accel_dma_rx_buf[SPI_DMA_ACCEL_LENGHT];
uint8_t accel_dma_tx_buf[SPI_DMA_ACCEL_LENGHT] = {0x92,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
uint8_t accel_temp_dma_rx_buf[SPI_DMA_ACCEL_TEMP_LENGHT];
uint8_t accel_temp_dma_tx_buf[SPI_DMA_ACCEL_TEMP_LENGHT] = {0xA2,0xFF,0xFF,0xFF};
	/*(6)��־λ*/
volatile uint8_t gyro_update_flag = 0;
volatile uint8_t accel_update_flag = 0;
volatile uint8_t accel_temp_update_flag = 0;
volatile uint8_t mag_update_flag = 0;
volatile uint8_t imu_start_dma_flag = 0;
volatile uint8_t imu_read_flag = 0;
	/*(7)INS����*/
static fp32 INS_gyro[3] = {0.0f, 0.0f, 0.0f};
fp32 INS_accel[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_mag[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_quat[4] = {0.0f, 0.0f, 0.0f, 0.0f};
fp32 INS_angle[3] = {0.0f, 0.0f, 0.0f};      //euler angle, unit rad.ŷ���� ��λ rad
fp32 INS_angle_deg[3] = {0.0f, 0.0f, 0.0f};
	/*(8)��Ư��ƫ�ü������*/
#define BMI088_BOARD_INSTALL_SPIN_MATRIX    \
    {-1.0f, 0.0f, 0.0f},                     \
    {0.0f, -1.0f, 0.0f},                     \
    {0.0f, 0.0f, 1.0f}                     \


#define IST8310_BOARD_INSTALL_SPIN_MATRIX   \
    {1.0f, 0.0f, 0.0f},                     \
    {0.0f, 1.0f, 0.0f},                     \
    {0.0f, 0.0f, 1.0f}                      \
		
fp32 gyro_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
fp32 gyro_offset[3];
fp32 gyro_cali_offset[3];
fp32 accel_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
fp32 accel_offset[3];
fp32 accel_cali_offset[3];
fp32 mag_scale_factor[3][3] = {IST8310_BOARD_INSTALL_SPIN_MATRIX};
fp32 mag_offset[3];
fp32 mag_cali_offset[3];
	/*(9)���ٶȼƵ�ͨ�˲�*/
static fp32 accel_fliter_1[3] = {0.0f, 0.0f, 0.0f};
static fp32 accel_fliter_2[3] = {0.0f, 0.0f, 0.0f};
static fp32 accel_fliter_3[3] = {0.0f, 0.0f, 0.0f};
static const fp32 fliter_num[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};

static const float timing_time = 0.001f;   //tast run time , unit s.�������е�ʱ�� ��λ s








/*****************************[1]	���߳���********************************************/
/*[1.1]
 * brief			��ʼ��bmi088��ist8310,ͨ��DMA��ȡ���ݣ���̬�������ŷ����
 * postscript		*/
void INS_task(void const *pvParameters)
{
		/**(1)	����տ�ʼ����ʱһ��ʱ��**/
    osDelay(INS_TASK_INIT_TIME);
	
		/**(2)	��ʼ��BMI088��ist8310**/
		while(BMI088_init())	{	osDelay(100);	}//��ʼ�����ɹ���һֱ��ʼ��
		while(ist8310_init())	{	osDelay(100);	}
		
		/**(3)	�ȶ�һ��BMI088��ֵ**/
    BMI088_read(bmi088_real_data.gyro, bmi088_real_data.accel, &bmi088_real_data.temp);
		
    /**(4)	��ת��������Ư**/
    imu_cali_slove(INS_gyro, INS_accel, INS_mag, &bmi088_real_data, &ist8310_real_data);
		
		/**(5)	��ʼ���¶ȿ���PID(��ֹ�¶�Ư�ƣ�)**/
    PID_init(&imu_temp_pid, PID_POSITION, imu_temp_PID, TEMPERATURE_PID_MAX_OUT, TEMPERATURE_PID_MAX_IOUT);
		
		/**(6)	��ʼ����Ԫ��**/
    AHRS_init(INS_quat, INS_accel, INS_mag);

		/**(7)	��ȡ��ǰ����ľ��(��Ϊ����һ�������ѹ���)**/
    INS_Task_local_handler = xTaskGetHandle(pcTaskGetName(NULL));

		/**(8)	SPI����DMA��ʼ��**/
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;//����SPIƵ��
    if (HAL_SPI_Init(&hspi1) != HAL_OK)Error_Handler();//cube���ɴ����ʱ��û�г�ʼ����
		imu_start_dma_flag = 1;//������DMA���䣬��־������1
    SPI1_DMA_init((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);//��ʼ����ʱ�������ǵĻ���ռ䣿

    /**(9)	����ƫ�ò�����**/
		bmi088_offset_data.gyro[0] =0.00293904217f; 
		bmi088_offset_data.gyro[1] =-0.00386332953f;
		bmi088_offset_data.gyro[2] =-3.5153731e-05f;
		
//		mpu_offset_clc();
    
    while (1)
    {
				/*(1)	�ȴ�SPI DMA������ɺ����񱻻��ѣ�����һֱ����while����*/
        while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY)!= pdPASS)
        {
        }

				/**(2)	��DMA�����BMI088�����������ݸ���BMI088�ṹ����Ӧ��Ա����**/
				/*(2.1)	������������ݸ������*/
        if(gyro_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
					/*(2.1.1)	��Ӧ��־λ����*/
            gyro_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
					/*(2.1.2)	��������ԭʼ���ݴ�����Ϊʵ��������ֵ�������ṹ���Ա����*/
            BMI088_gyro_read_over(gyro_dma_rx_buf + BMI088_GYRO_RX_BUF_DATA_OFFSET, bmi088_real_data.gyro);
					/*(2.1.3)	��ȥƫ��*/
						bmi088_real_data.gyro[0] -= bmi088_offset_data.gyro[0];
						bmi088_real_data.gyro[1] -= bmi088_offset_data.gyro[1];
						bmi088_real_data.gyro[2] -= bmi088_offset_data.gyro[2];
        }
				/*(2.2)	������ٶȼ����ݸ������*/
        if(accel_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
					/*(2.2.1)	��Ӧ��־λ����*/
            accel_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
					/*(2.2.2)	�����ٶȼ�ԭʼ���ݴ�����Ϊʵ��������ֵ�������ṹ���Ա����*/
            BMI088_accel_read_over(accel_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, bmi088_real_data.accel, &bmi088_real_data.time);
        }
				/*(2.3)	����¶����ݸ������*/
        if(accel_temp_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
						/*(2.3.1)	��Ӧ��־λ����*/
            accel_temp_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
						/*(2.3.2)	���¶�ԭʼ���ݴ�����Ϊʵ��������ֵ�������ṹ���Ա����*/
            BMI088_temperature_read_over(accel_temp_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, &bmi088_real_data.temp);
          	/*(2.3.3)	�����¶�PID����*/  
						imu_temp_control(bmi088_real_data.temp);
        }

        /**(3)	��ת��������Ư**/
        imu_cali_slove(INS_gyro, INS_accel, INS_mag, &bmi088_real_data, &ist8310_real_data);


				/**(4)	���ٶȼƵ�ͨ�˲�**/
        accel_fliter_1[0] = accel_fliter_2[0];
        accel_fliter_2[0] = accel_fliter_3[0];

        accel_fliter_3[0] = accel_fliter_2[0] * fliter_num[0] + accel_fliter_1[0] * fliter_num[1] + INS_accel[0] * fliter_num[2];

        accel_fliter_1[1] = accel_fliter_2[1];
        accel_fliter_2[1] = accel_fliter_3[1];

        accel_fliter_3[1] = accel_fliter_2[1] * fliter_num[0] + accel_fliter_1[1] * fliter_num[1] + INS_accel[1] * fliter_num[2];

        accel_fliter_1[2] = accel_fliter_2[2];
        accel_fliter_2[2] = accel_fliter_3[2];

        accel_fliter_3[2] = accel_fliter_2[2] * fliter_num[0] + accel_fliter_1[2] * fliter_num[1] + INS_accel[2] * fliter_num[2];

			/*(5)	��Ԫ�����º���̬����*/
			AHRS_update(INS_quat, timing_time, INS_gyro, accel_fliter_3, INS_mag);
			get_angle(INS_quat, INS_angle + INS_YAW_ADDRESS_OFFSET, INS_angle + INS_PITCH_ADDRESS_OFFSET, INS_angle + INS_ROLL_ADDRESS_OFFSET);
			
			/*(6)	��̬�� ����ת�Ƕ�*/
			INS_angle_deg[0] = INS_angle[0] * 180.0f / 3.141592653589f;
			INS_angle_deg[1] = INS_angle[1] * 180.0f / 3.141592653589f;
			INS_angle_deg[2] = INS_angle[2] * 180.0f / 3.141592653589f;


    }
}
/*[1.2]				
 * brief			GPIO�ⲿ�ж� �����￪��DMA�����������
 * postscript		�����Ǻͼ��ٶȼƶ��ᶨʱ�����½������巢�����ݣ����ﻹ���⿪��һ��GPIO_PIN_0���ж����ڻ�������
 					������һֱ�ڵȴ������ѣ��ڻ���֮ǰҪͨ���½����ⲿ�ж϶�ȡ���������ݣ�
					����������Ҫ�ȵ�GPIO0�����ⲿ�жϣ�����GPIO0�����ⲿ�ж��жϣ���Ҫ���������DMA���ݴ������DMA��������ж��������GPIO0
					Ҳ����˵���GPIO�жϺ�����DMA��������жϺ���������ϵģ����й�ϵҪ����� 			
					���У�������Ȼ�д����Ƶ����ݵ���û�ж�ȡ�����Ƶ�ֵ����͵��������������̲�һ��������ÿ�ζ��������Ǵ�0��ʼ*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
		/**(1)	����Ǽ��ٶȼƵ��½����ж�**/
    if(GPIO_Pin == INT1_ACCEL_Pin)
    {
					/*(1.1)	���ٶȼ�(�����¶ȴ�����)�Ľ������ݱ�־λ��1*/
        accel_update_flag |= 1 << IMU_DR_SHFITS;
        accel_temp_update_flag |= 1 << IMU_DR_SHFITS;
					/*(1.2)	����SPI��DMA����*/
        if(imu_start_dma_flag)imu_cmd_spi_dma();
    }
			/**(2)	����������ǵ��½����ж�**/
    else if(GPIO_Pin == INT1_GYRO_Pin)
    {
        gyro_update_flag |= 1 << IMU_DR_SHFITS;
        if(imu_start_dma_flag) imu_cmd_spi_dma();
        
    }
		
	/**(3)	�����ist8310���½����ж�**/
    else if(GPIO_Pin == DRDY_IST8310_Pin)
    {
			/*(3.1)	�����ƵĽ������ݱ�־λ��1*/
        mag_update_flag |= 1 << IMU_DR_SHFITS;//����û�ж�����
    }
		
	/**(4)	�����GPIO0�ⲿ�жϣ��Ǿ��Ǹû���������**/
	//����ⲿ�ж�Ҫ�ȵ�������DMA������ɺ���DMA��������ж���������������ڻ�������֮ǰ�������ǵ�����һ���Ǹ�����ɵ�
    else if(GPIO_Pin == GPIO_PIN_0)
    {
				imu_read_flag=1;
        //wake up the task
        //��������
        if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
        {
            static BaseType_t xHigherPriorityTaskWoken;
            vTaskNotifyGiveFromISR(INS_Task_local_handler, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }

    }
}
/*[1.3]				
 * brief			DMA������ɺ�����ж�
 * postscript		�����־λ����������ȡ��Ƭѡ��������������ݴ�����ɣ���Ҫ����GPIO0���ⲿ�жϣ�����[2]�ⲿ�ж����棬��Ҫ������������������				*/
void DMA2_Stream2_IRQHandler(void)
{

    if(__HAL_DMA_GET_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx)) != RESET)
    {
			/**(1)	���������ɱ�־λ**/
        __HAL_DMA_CLEAR_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx));

       /**(2)	��������Ƕ�ȡ���,SPI������־λ��0�����ݸ��±�־λ��1��ȡ��������Ƭѡ**/
        if(gyro_update_flag & (1 << IMU_SPI_SHFITS))
        {
					/*(2.1)	SPI������־λ����*/
            gyro_update_flag &= ~(1 << IMU_SPI_SHFITS);
					/*(2.2)	���ݸ��±�־λ��1*/
            gyro_update_flag |= (1 << IMU_UPDATE_SHFITS);
					/*(2.3)	ȡ�� ������Ƭѡ*/
            HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
        }

			/**(3)	������ٶȼƶ�ȡ���,SPI������־λ��0�����ݸ��±�־λ��1��ȡ�����ٶȼ�Ƭѡ**/
        if(accel_update_flag & (1 << IMU_SPI_SHFITS))
        {
					/*(3.1)	SPI������־λ����*/
            accel_update_flag &= ~(1 << IMU_SPI_SHFITS);
					/*(3.2)	���ݸ��±�־λ��1*/
            accel_update_flag |= (1 << IMU_UPDATE_SHFITS);
					/*(3.3)	ȡ�� ���ٶȼ�Ƭѡ*/
            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
        }
        /**(4)	����¶ȶ�ȡ��ϣ��¶ȴ��������ڼ��ٶȼ����棩,SPI������־λ��0�����ݸ��±�־λ��1��ȡ�����ٶȼ�Ƭѡ**/
        if(accel_temp_update_flag & (1 << IMU_SPI_SHFITS))
        {
					/*(4.1)	SPI������־λ����*/
            accel_temp_update_flag &= ~(1 << IMU_SPI_SHFITS);
					/*(4.2)	���ݸ��±�־λ��1*/
            accel_temp_update_flag |= (1 << IMU_UPDATE_SHFITS);
					/*(4.3)	ȡ�� �¶ȴ�����(���ٶȼ�)Ƭѡ*/
            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
        }
				
       	/**(5)	Ϊʲô��Ҫ�������ٿ���һ�Σ���ִ����**/
        imu_cmd_spi_dma();
				/**(6)	������������ݴ�����ϣ�����GPIO0���ⲿ�жϣ��������ת���ⲿ�жϺ�����ִ�л����������**/
        if(gyro_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
					/*(6.1)	����GPIO0���ⲿ�ж�*/
            __HAL_GPIO_EXTI_GENERATE_SWIT(GPIO_PIN_0);//ò��û�й������һ
        }
    }
}


/*****************************[2]	imu���ƺ�������********************************************/

/**[2.1]				��[1.2]��[1.3]�е���
  *	brief			����xxx_update_flag�ĵ�0λ����������Ӧ��Ƭѡ��SPI��DMA
  * postscript 				*/
static void imu_cmd_spi_dma(void)
{

				/**(1)	���������ǵ�DMA����**/
        if( (gyro_update_flag & (1 << IMU_DR_SHFITS) ) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
        && !(accel_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)))
        {
						/*(1.1)	�޸ı�־λ*/
            gyro_update_flag &= ~(1 << IMU_DR_SHFITS);
            gyro_update_flag |= (1 << IMU_SPI_SHFITS);
						/*(1.2)	Ƭѡѡ�������ǲ�����������SPI��DMA����*/
            HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
            SPI1_DMA_enable((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);
            return;
        }
				/**(2)	�������ٶȼƵ�DMA����**/
        if((accel_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
        && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)))
        {
					/*(2.1)	�޸ı�־λ*/
            accel_update_flag &= ~(1 << IMU_DR_SHFITS);
            accel_update_flag |= (1 << IMU_SPI_SHFITS);
					/*(2.2)	Ƭѡѡ�м��ٶȼƲ��������ٶȼ�SPI��DMA����*/
            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
            SPI1_DMA_enable((uint32_t)accel_dma_tx_buf, (uint32_t)accel_dma_rx_buf, SPI_DMA_ACCEL_LENGHT);
            return;
        }
        	/**(3)	�����¶ȴ�������DMA����**/
        if((accel_temp_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
        && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_update_flag & (1 << IMU_SPI_SHFITS)))
        {
					/*(3.1)	�޸ı�־λ*/
            accel_temp_update_flag &= ~(1 << IMU_DR_SHFITS);
            accel_temp_update_flag |= (1 << IMU_SPI_SHFITS);
					/*(3.2)	Ƭѡѡ���¶ȴ�����(�ڼ��ٶȼ�)�������¶ȴ�����SPI��DMA����*/
            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
            SPI1_DMA_enable((uint32_t)accel_temp_dma_tx_buf, (uint32_t)accel_temp_dma_rx_buf, SPI_DMA_ACCEL_TEMP_LENGHT);
            return;
        }
}

 /**[2.2]				��whlie(1)�����¶�����ʱ����
  *	brief				����bmi088���¶�
  * param[in]		��ǰ���¶�
  * postscript 				*/
static void imu_temp_control(fp32 temp)
{
    uint16_t tempPWM;
    static uint8_t temp_constant_time = 0;
	
	/*(3)	�ﵽĿ���¶Ⱥ����PID�¶ȿ���*/
    if (first_temperate)
    {
				/*(3.1)	PID����PWM���ֵ*/
        PID_calc(&imu_temp_pid, temp, IMU_Temp_Set);
			/*(3.2)	������ֵС���㣬PWMû����С���㣬����㲻����*/
        if (imu_temp_pid.out < 0.0f)    imu_temp_pid.out = 0.0f;
        
				/*(3.3)	pid���ֵ����PWM����������PWM������*/
        tempPWM = (uint16_t)imu_temp_pid.out;
        IMU_temp_PWM(tempPWM);
    }
    else
    {
       /*(2)	�ﵽ�趨�¶Ȳ�����200���������ڣ�������������Ϊ����ʵ�һ�룬������������(���Ѹ�ټ��ȳ���ִֻ��һ�Σ����濿PID�¿أ�������temp_constant_time��û����)*/
        if (temp > IMU_Temp_Set)
        {
					/*(2.1)	Ҫ�����趨�¶�һ��ʱ�������Ϊ��ȫ���趨�¶�*/
            temp_constant_time++;
            if (temp_constant_time > 200)
            {
							/*(2.2)	�������µ�һ�δﵽĿ���¶ȣ������Ǹ�if�ж�Ϊ���ˣ��Ժ��ִ��PID�¿س���*/
                first_temperate = 1;
							/*(2.3)	������������Ϊ����ʵ�һ�룬��������*/
                imu_temp_pid.Iout = MPU6500_TEMP_PWM_MAX / 2.0f;
            }
        }
				/*(1)	û�дﵽ�趨�¶�һֱ������ʼ���,���ٵ�Ŀ���¶ȸ���*/
        IMU_temp_PWM(MPU6500_TEMP_PWM_MAX - 1);
    }
}

/**[2.3]
  * @brief          ��ת������,���ٶȼƺʹ�����,��������Ư,��Ϊ�豸�в�ͬ��װ��ʽ
  * @param[out]     gyro: ������Ư����ת
  * @param[out]     accel: ������Ư����ת
  * @param[out]     mag: ������Ư����ת
  * @param[in]      bmi088: �����Ǻͼ��ٶȼ�����
  * @param[in]      ist8310: ����������
  * @retval         none
  */
static void imu_cali_slove(fp32 gyro[3], fp32 accel[3], fp32 mag[3], bmi088_real_data_t *bmi088, ist8310_real_data_t *ist8310)
{
    for (uint8_t i = 0; i < 3; i++)
    {
        gyro[i] = bmi088->gyro[0] * gyro_scale_factor[i][0] + bmi088->gyro[1] * gyro_scale_factor[i][1] + bmi088->gyro[2] * gyro_scale_factor[i][2];
        accel[i] = bmi088->accel[0] * accel_scale_factor[i][0] + bmi088->accel[1] * accel_scale_factor[i][1] + bmi088->accel[2] * accel_scale_factor[i][2] + accel_offset[i];
        mag[i] = ist8310->mag[0] * mag_scale_factor[i][0] + ist8310->mag[1] * mag_scale_factor[i][1] + ist8310->mag[2] * mag_scale_factor[i][2] + mag_offset[i];
    }
}



/*****************************[3]	��Щ������û�õ�********************************************/
/**
  * @brief          ������������Ư
  * @param[out]     gyro_offset:������Ư
  * @param[in]      gyro:���ٶ�����
  * @param[out]     offset_time_count: �Զ���1
  * @retval         none
  */
void gyro_offset_calc(fp32 gyro_offset[3], fp32 gyro[3], uint16_t *offset_time_count)
{
    if (gyro_offset == NULL || gyro == NULL || offset_time_count == NULL)
    {
        return;
    }

        gyro_offset[0] = gyro_offset[0] + 0.00005f * gyro[0];
        gyro_offset[1] = gyro_offset[1] + 0.00005f * gyro[1];
        gyro_offset[2] = gyro_offset[2] + 0.00005f * gyro[2];
        (*offset_time_count)++;
}


/**
  * @brief          У׼������
  * @param[out]     �����ǵı������ӣ�1.0fΪĬ��ֵ�����޸�
  * @param[out]     �����ǵ���Ư���ɼ������ǵľ�ֹ�������Ϊoffset
  * @param[out]     �����ǵ�ʱ�̣�ÿ����gyro_offset���û��1,
  * @retval         none
  */
void INS_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[3], uint16_t *time_count)
{
        if( *time_count == 0)
        {
            gyro_offset[0] = gyro_cali_offset[0];
            gyro_offset[1] = gyro_cali_offset[1];
            gyro_offset[2] = gyro_cali_offset[2];
        }
        gyro_offset_calc(gyro_offset, INS_gyro, time_count);

        cali_offset[0] = gyro_offset[0];
        cali_offset[1] = gyro_offset[1];
        cali_offset[2] = gyro_offset[2];
        cali_scale[0] = 1.0f;
        cali_scale[1] = 1.0f;
        cali_scale[2] = 1.0f;

}

/**
  * @brief          У׼���������ã�����flash���������ط�����У׼ֵ
  * @param[in]      �����ǵı������ӣ�1.0fΪĬ��ֵ�����޸�
  * @param[in]      �����ǵ���Ư
  * @retval         none
  */
void INS_set_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[3])
{
    gyro_cali_offset[0] = cali_offset[0];
    gyro_cali_offset[1] = cali_offset[1];
    gyro_cali_offset[2] = cali_offset[2];
    gyro_offset[0] = gyro_cali_offset[0];
    gyro_offset[1] = gyro_cali_offset[1];
    gyro_offset[2] = gyro_cali_offset[2];
}

/**
  * @brief          ��ȡ��Ԫ��
  * @param[in]      none
  * @retval         INS_quat��ָ��
  */
const fp32 *get_INS_quat_point(void)
{
    return INS_quat;
}

/**
  * @brief          ��ȡŷ����, 0:yaw, 1:pitch, 2:roll ��λ rad
  * @param[in]      none
  * @retval         INS_angle��ָ��
  */
const fp32 *get_INS_angle_point(void)
{
    return INS_angle;
}


/**
  * @brief          ��ȡ���ٶ�,0:x��, 1:y��, 2:roll�� ��λ rad/s
  * @param[in]      none
  * @retval         INS_gyro��ָ��
  */
extern const fp32 *get_gyro_data_point(void)
{
    //return INS_gyro;
		return bmi088_real_data.gyro;
}

/**
  * @brief          ��ȡ���ٶ�,0:x��, 1:y��, 2:roll�� ��λ m/s2
  * @param[in]      none
  * @retval         INS_accel��ָ��
  */
extern const fp32 *get_accel_data_point(void)
{
    return INS_accel;
}

/**
  * @brief          ��ȡ���ٶ�,0:x��, 1:y��, 2:roll�� ��λ ut
  * @param[in]      none
  * @retval         INS_mag��ָ��
  */
extern const fp32 *get_mag_data_point(void)
{
    return INS_mag;
}


/**
  * @brief          ����IMU��Ư
  * @param[in]      none
  * @retval         none
  */
void mpu_offset_clc(void)
{
	static uint16_t i =0 ,j=0;
	while( i < 3000 || j < 3000 )
	{
			while (imu_read_flag == 0)
			{
			}
			imu_read_flag = 0;
			if(gyro_update_flag & (1 << IMU_UPDATE_SHFITS))
			{
				if(i<3000)
				{
					gyro_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
					BMI088_gyro_read_over(gyro_dma_rx_buf + BMI088_GYRO_RX_BUF_DATA_OFFSET, bmi088_real_data.gyro);
					bmi088_offset_data.gyro[0] += bmi088_real_data.gyro[0];
					bmi088_offset_data.gyro[1] += bmi088_real_data.gyro[1];
					bmi088_offset_data.gyro[2] += bmi088_real_data.gyro[2];
				}	
				else if(i==3000)
				{
					bmi088_offset_data.gyro[0] = bmi088_offset_data.gyro[0] / 3000.0f;
					bmi088_offset_data.gyro[1] = bmi088_offset_data.gyro[1] / 3000.0f;
					bmi088_offset_data.gyro[2] = bmi088_offset_data.gyro[2] / 3000.0f;
				}
				i++;		
			}

			if(accel_update_flag & (1 << IMU_UPDATE_SHFITS))
			{
				if(j<3000)
				{
					accel_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
					BMI088_accel_read_over(accel_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, bmi088_real_data.accel, &bmi088_real_data.time);
					bmi088_offset_data.accel[0] += bmi088_real_data.accel[0];
					bmi088_offset_data.accel[1] += bmi088_real_data.accel[1];
					bmi088_offset_data.accel[2] += bmi088_real_data.accel[2];
				}	
				else if(j==3000)
				{
					bmi088_offset_data.accel[0] = bmi088_offset_data.accel[0] / 3000.0f;
					bmi088_offset_data.accel[1] = bmi088_offset_data.accel[1] / 3000.0f;
					bmi088_offset_data.accel[2] = bmi088_offset_data.accel[2] / 3000.0f;
				}
				j++;
			}
	}
}


