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
主要任务：	 通过调度SPI的DMA传输读取BMI088传感器数据；
						 直接用CPU读取ist8310传感器数据；
						 用传感器数据进行姿态解算				
																											唤醒任务							已完成xxx的SPI的DMA传输		开启xxx的SPI的DMA传输			标志进入下降沿外部中断				
				程序运行位置							程序运行阶段		xxx_update_flag的BIT[3]		xxx_update_flag的BIT[2]		xxx_update_flag的BIT[1]		xxx_update_flag的BIT[0]
					初始化 																						0													0													0													0
															SPI接收到传感器数据					 
				进入外部中断																		   	0													0													0													1
																开启SPI的DMA传输				    0													0													1													0
																………………(DMA传输中)
																完成SPI的DMA传输												
			进入DMA传输结束中断																		0													1													0													0
														(如果是陀螺仪传输结束)
															开启外部中断Line_0						1													0													0													0
				进入外部中断
															唤醒while(1)的任务
				进入while(1)
															转移数据，姿态解算
*/






/**
  * @brief          旋转陀螺仪,加速度计和磁力计,并计算零漂,因为设备有不同安装方式
  * @param[out]     gyro: 加上零漂和旋转
  * @param[out]     accel: 加上零漂和旋转
  * @param[out]     mag: 加上零漂和旋转
  * @param[in]      bmi088: 陀螺仪和加速度计数据
  * @param[in]      ist8310: 磁力计数据
  * @retval         none
  */
static void imu_cali_slove(fp32 gyro[3], fp32 accel[3], fp32 mag[3], bmi088_real_data_t *bmi088, ist8310_real_data_t *ist8310);

/**
  * @brief          控制bmi088的温度
  * @param[in]      temp:bmi088的温度
  * @retval         none
  */
static void imu_temp_control(fp32 temp);

/**
  * @brief          根据imu_update_flag的值开启SPI DMA
  * @param[in]      temp:bmi088的温度
  * @retval         none
  */
static void imu_cmd_spi_dma(void);

/**
  * @brief          计算IMU零漂
  * @param[in]      none
  * @retval         none
  */
void mpu_offset_clc(void);



	/*(1)传感器数据存储结构体变量*/
bmi088_real_data_t bmi088_real_data;
bmi088_real_data_t bmi088_offset_data;//误差数据
ist8310_real_data_t ist8310_real_data;
	/*(2)温度pid初始化和温度控制参数*/
static uint8_t first_temperate;
static const fp32 imu_temp_PID[3] = {TEMPERATURE_PID_KP, TEMPERATURE_PID_KI, TEMPERATURE_PID_KD};
static pid_type_def imu_temp_pid;
#define IMU_temp_PWM(pwm)  imu_pwm_set(pwm)  
	/*(3)任务句柄*/
static TaskHandle_t INS_Task_local_handler;
	/*(4)SPI的句柄*/
extern SPI_HandleTypeDef hspi1;
	/*(5)DMA接收和发送缓存*/
uint8_t gyro_dma_rx_buf[SPI_DMA_GYRO_LENGHT];
uint8_t gyro_dma_tx_buf[SPI_DMA_GYRO_LENGHT] = {0x82,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
uint8_t accel_dma_rx_buf[SPI_DMA_ACCEL_LENGHT];
uint8_t accel_dma_tx_buf[SPI_DMA_ACCEL_LENGHT] = {0x92,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
uint8_t accel_temp_dma_rx_buf[SPI_DMA_ACCEL_TEMP_LENGHT];
uint8_t accel_temp_dma_tx_buf[SPI_DMA_ACCEL_TEMP_LENGHT] = {0xA2,0xFF,0xFF,0xFF};
	/*(6)标志位*/
volatile uint8_t gyro_update_flag = 0;
volatile uint8_t accel_update_flag = 0;
volatile uint8_t accel_temp_update_flag = 0;
volatile uint8_t mag_update_flag = 0;
volatile uint8_t imu_start_dma_flag = 0;
volatile uint8_t imu_read_flag = 0;
	/*(7)INS变量*/
static fp32 INS_gyro[3] = {0.0f, 0.0f, 0.0f};
fp32 INS_accel[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_mag[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_quat[4] = {0.0f, 0.0f, 0.0f, 0.0f};
fp32 INS_angle[3] = {0.0f, 0.0f, 0.0f};      //euler angle, unit rad.欧拉角 单位 rad
fp32 INS_angle_deg[3] = {0.0f, 0.0f, 0.0f};
	/*(8)零漂和偏置计算变量*/
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
	/*(9)加速度计低通滤波*/
static fp32 accel_fliter_1[3] = {0.0f, 0.0f, 0.0f};
static fp32 accel_fliter_2[3] = {0.0f, 0.0f, 0.0f};
static fp32 accel_fliter_3[3] = {0.0f, 0.0f, 0.0f};
static const fp32 fliter_num[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};

static const float timing_time = 0.001f;   //tast run time , unit s.任务运行的时间 单位 s








/*****************************[1]	主线程序********************************************/
/*[1.1]
 * brief			初始化bmi088和ist8310,通过DMA获取数据，姿态解算计算欧拉角
 * postscript		*/
void INS_task(void const *pvParameters)
{
		/**(1)	任务刚开始先延时一段时间**/
    osDelay(INS_TASK_INIT_TIME);
	
		/**(2)	初始化BMI088和ist8310**/
		while(BMI088_init())	{	osDelay(100);	}//初始化不成功就一直初始化
		while(ist8310_init())	{	osDelay(100);	}
		
		/**(3)	先读一下BMI088的值**/
    BMI088_read(bmi088_real_data.gyro, bmi088_real_data.accel, &bmi088_real_data.temp);
		
    /**(4)	旋转并计算零漂**/
    imu_cali_slove(INS_gyro, INS_accel, INS_mag, &bmi088_real_data, &ist8310_real_data);
		
		/**(5)	初始化温度控制PID(防止温度漂移？)**/
    PID_init(&imu_temp_pid, PID_POSITION, imu_temp_PID, TEMPERATURE_PID_MAX_OUT, TEMPERATURE_PID_MAX_IOUT);
		
		/**(6)	初始化四元数**/
    AHRS_init(INS_quat, INS_accel, INS_mag);

		/**(7)	获取当前任务的句柄(因为加了一个任务唤醒功能)**/
    INS_Task_local_handler = xTaskGetHandle(pcTaskGetName(NULL));

		/**(8)	SPI及其DMA初始化**/
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;//设置SPI频率
    if (HAL_SPI_Init(&hspi1) != HAL_OK)Error_Handler();//cube生成代码的时候没有初始化吗
		imu_start_dma_flag = 1;//允许开启DMA传输，标志变量置1
    SPI1_DMA_init((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);//初始化暂时用陀螺仪的缓存空间？

    /**(9)	设置偏置参数？**/
		bmi088_offset_data.gyro[0] =0.00293904217f; 
		bmi088_offset_data.gyro[1] =-0.00386332953f;
		bmi088_offset_data.gyro[2] =-3.5153731e-05f;
		
//		mpu_offset_clc();
    
    while (1)
    {
				/*(1)	等待SPI DMA传输完成后任务被唤醒，否则一直卡在while里面*/
        while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY)!= pdPASS)
        {
        }

				/**(2)	将DMA传输的BMI088传感器的数据赋给BMI088结构体相应成员变量**/
				/*(2.1)	如果陀螺仪数据更新完毕*/
        if(gyro_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
					/*(2.1.1)	相应标志位归零*/
            gyro_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
					/*(2.1.2)	将陀螺仪原始数据处理后变为实际物理量值，赋给结构体成员变量*/
            BMI088_gyro_read_over(gyro_dma_rx_buf + BMI088_GYRO_RX_BUF_DATA_OFFSET, bmi088_real_data.gyro);
					/*(2.1.3)	减去偏置*/
						bmi088_real_data.gyro[0] -= bmi088_offset_data.gyro[0];
						bmi088_real_data.gyro[1] -= bmi088_offset_data.gyro[1];
						bmi088_real_data.gyro[2] -= bmi088_offset_data.gyro[2];
        }
				/*(2.2)	如果加速度计数据更新完毕*/
        if(accel_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
					/*(2.2.1)	相应标志位归零*/
            accel_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
					/*(2.2.2)	将加速度计原始数据处理后变为实际物理量值，赋给结构体成员变量*/
            BMI088_accel_read_over(accel_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, bmi088_real_data.accel, &bmi088_real_data.time);
        }
				/*(2.3)	如果温度数据更新完毕*/
        if(accel_temp_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
						/*(2.3.1)	相应标志位归零*/
            accel_temp_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
						/*(2.3.2)	将温度原始数据处理后变为实际物理量值，赋给结构体成员变量*/
            BMI088_temperature_read_over(accel_temp_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, &bmi088_real_data.temp);
          	/*(2.3.3)	开启温度PID控制*/  
						imu_temp_control(bmi088_real_data.temp);
        }

        /**(3)	旋转并计算零漂**/
        imu_cali_slove(INS_gyro, INS_accel, INS_mag, &bmi088_real_data, &ist8310_real_data);


				/**(4)	加速度计低通滤波**/
        accel_fliter_1[0] = accel_fliter_2[0];
        accel_fliter_2[0] = accel_fliter_3[0];

        accel_fliter_3[0] = accel_fliter_2[0] * fliter_num[0] + accel_fliter_1[0] * fliter_num[1] + INS_accel[0] * fliter_num[2];

        accel_fliter_1[1] = accel_fliter_2[1];
        accel_fliter_2[1] = accel_fliter_3[1];

        accel_fliter_3[1] = accel_fliter_2[1] * fliter_num[0] + accel_fliter_1[1] * fliter_num[1] + INS_accel[1] * fliter_num[2];

        accel_fliter_1[2] = accel_fliter_2[2];
        accel_fliter_2[2] = accel_fliter_3[2];

        accel_fliter_3[2] = accel_fliter_2[2] * fliter_num[0] + accel_fliter_1[2] * fliter_num[1] + INS_accel[2] * fliter_num[2];

			/*(5)	四元数更新和姿态解算*/
			AHRS_update(INS_quat, timing_time, INS_gyro, accel_fliter_3, INS_mag);
			get_angle(INS_quat, INS_angle + INS_YAW_ADDRESS_OFFSET, INS_angle + INS_PITCH_ADDRESS_OFFSET, INS_angle + INS_ROLL_ADDRESS_OFFSET);
			
			/*(6)	姿态角 弧度转角度*/
			INS_angle_deg[0] = INS_angle[0] * 180.0f / 3.141592653589f;
			INS_angle_deg[1] = INS_angle[1] * 180.0f / 3.141592653589f;
			INS_angle_deg[2] = INS_angle[2] * 180.0f / 3.141592653589f;


    }
}
/*[1.2]				
 * brief			GPIO外部中断 在这里开启DMA传输或唤醒任务
 * postscript		陀螺仪和加速度计都会定时产生下降沿脉冲发送数据；这里还额外开了一个GPIO_PIN_0的中断用于唤醒任务
 					主任务一直在等待任务唤醒，在唤醒之前要通过下降沿外部中断读取传感器数据，
					唤醒任务需要等到GPIO0产生外部中断，而让GPIO0产生外部中断中断，需要陀螺仪完成DMA数据传输进入DMA传输完成中断里面操作GPIO0
					也就是说这个GPIO中断函数和DMA传输完成中断函数存在耦合的，个中关系要理清楚 			
					还有，这里虽然有磁力计的内容但是没有读取磁力计的值，这和单独的陀螺仪例程不一样，否则每次读出来不是从0开始*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
		/**(1)	如果是加速度计的下降沿中断**/
    if(GPIO_Pin == INT1_ACCEL_Pin)
    {
					/*(1.1)	加速度计(包括温度传感器)的接收数据标志位置1*/
        accel_update_flag |= 1 << IMU_DR_SHFITS;
        accel_temp_update_flag |= 1 << IMU_DR_SHFITS;
					/*(1.2)	开启SPI的DMA传输*/
        if(imu_start_dma_flag)imu_cmd_spi_dma();
    }
			/**(2)	如果是陀螺仪的下降沿中断**/
    else if(GPIO_Pin == INT1_GYRO_Pin)
    {
        gyro_update_flag |= 1 << IMU_DR_SHFITS;
        if(imu_start_dma_flag) imu_cmd_spi_dma();
        
    }
		
	/**(3)	如果是ist8310的下降沿中断**/
    else if(GPIO_Pin == DRDY_IST8310_Pin)
    {
			/*(3.1)	磁力计的接收数据标志位置1*/
        mag_update_flag |= 1 << IMU_DR_SHFITS;//后面没有读数据
    }
		
	/**(4)	如果是GPIO0外部中断，那就是该唤醒任务了**/
	//这个外部中断要等到陀螺仪DMA传输完成后，在DMA传输完成中断里面操作，所以在唤醒任务之前，陀螺仪的数据一定是更新完成的
    else if(GPIO_Pin == GPIO_PIN_0)
    {
				imu_read_flag=1;
        //wake up the task
        //唤醒任务
        if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
        {
            static BaseType_t xHigherPriorityTaskWoken;
            vTaskNotifyGiveFromISR(INS_Task_local_handler, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }

    }
}
/*[1.3]				
 * brief			DMA传输完成后进入中断
 * postscript		清除标志位；拉高引脚取消片选；如果陀螺仪数据传输完成，就要开启GPIO0的外部中断，而在[2]外部中断里面，就要唤醒主任务处理数据了				*/
void DMA2_Stream2_IRQHandler(void)
{

    if(__HAL_DMA_GET_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx)) != RESET)
    {
			/**(1)	清除传输完成标志位**/
        __HAL_DMA_CLEAR_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx));

       /**(2)	如果陀螺仪读取完毕,SPI开启标志位置0，数据更新标志位置1，取消陀螺仪片选**/
        if(gyro_update_flag & (1 << IMU_SPI_SHFITS))
        {
					/*(2.1)	SPI开启标志位归零*/
            gyro_update_flag &= ~(1 << IMU_SPI_SHFITS);
					/*(2.2)	数据更新标志位置1*/
            gyro_update_flag |= (1 << IMU_UPDATE_SHFITS);
					/*(2.3)	取消 陀螺仪片选*/
            HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
        }

			/**(3)	如果加速度计读取完毕,SPI开启标志位置0，数据更新标志位置1，取消加速度计片选**/
        if(accel_update_flag & (1 << IMU_SPI_SHFITS))
        {
					/*(3.1)	SPI开启标志位归零*/
            accel_update_flag &= ~(1 << IMU_SPI_SHFITS);
					/*(3.2)	数据更新标志位置1*/
            accel_update_flag |= (1 << IMU_UPDATE_SHFITS);
					/*(3.3)	取消 加速度计片选*/
            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
        }
        /**(4)	如果温度读取完毕（温度传感器就在加速度计里面）,SPI开启标志位置0，数据更新标志位置1，取消加速度计片选**/
        if(accel_temp_update_flag & (1 << IMU_SPI_SHFITS))
        {
					/*(4.1)	SPI开启标志位归零*/
            accel_temp_update_flag &= ~(1 << IMU_SPI_SHFITS);
					/*(4.2)	数据更新标志位置1*/
            accel_temp_update_flag |= (1 << IMU_UPDATE_SHFITS);
					/*(4.3)	取消 温度传感器(加速度计)片选*/
            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
        }
				
       	/**(5)	为什么还要在这里再开启一次？会执行吗**/
        imu_cmd_spi_dma();
				/**(6)	如果陀螺仪数据传输完毕，开启GPIO0的外部中断，程序会跳转道外部中断函数，执行唤醒任务程序**/
        if(gyro_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
					/*(6.1)	开启GPIO0的外部中断*/
            __HAL_GPIO_EXTI_GENERATE_SWIT(GPIO_PIN_0);//貌似没有归零和置一
        }
    }
}


/*****************************[2]	imu控制函数定义********************************************/

/**[2.1]				在[1.2]和[1.3]中调用
  *	brief			根据xxx_update_flag的第0位决定开启对应的片选和SPI的DMA
  * postscript 				*/
static void imu_cmd_spi_dma(void)
{

				/**(1)	开启陀螺仪的DMA传输**/
        if( (gyro_update_flag & (1 << IMU_DR_SHFITS) ) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
        && !(accel_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)))
        {
						/*(1.1)	修改标志位*/
            gyro_update_flag &= ~(1 << IMU_DR_SHFITS);
            gyro_update_flag |= (1 << IMU_SPI_SHFITS);
						/*(1.2)	片选选中陀螺仪并开启陀螺仪SPI的DMA传输*/
            HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
            SPI1_DMA_enable((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);
            return;
        }
				/**(2)	开启加速度计的DMA传输**/
        if((accel_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
        && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)))
        {
					/*(2.1)	修改标志位*/
            accel_update_flag &= ~(1 << IMU_DR_SHFITS);
            accel_update_flag |= (1 << IMU_SPI_SHFITS);
					/*(2.2)	片选选中加速度计并开启加速度计SPI的DMA传输*/
            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
            SPI1_DMA_enable((uint32_t)accel_dma_tx_buf, (uint32_t)accel_dma_rx_buf, SPI_DMA_ACCEL_LENGHT);
            return;
        }
        	/**(3)	开启温度传感器的DMA传输**/
        if((accel_temp_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
        && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_update_flag & (1 << IMU_SPI_SHFITS)))
        {
					/*(3.1)	修改标志位*/
            accel_temp_update_flag &= ~(1 << IMU_DR_SHFITS);
            accel_temp_update_flag |= (1 << IMU_SPI_SHFITS);
					/*(3.2)	片选选中温度传感器(在加速度计)并开启温度传感器SPI的DMA传输*/
            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
            SPI1_DMA_enable((uint32_t)accel_temp_dma_tx_buf, (uint32_t)accel_temp_dma_rx_buf, SPI_DMA_ACCEL_TEMP_LENGHT);
            return;
        }
}

 /**[2.2]				在whlie(1)处理温度数据时调用
  *	brief				控制bmi088的温度
  * param[in]		当前的温度
  * postscript 				*/
static void imu_temp_control(fp32 temp)
{
    uint16_t tempPWM;
    static uint8_t temp_constant_time = 0;
	
	/*(3)	达到目标温度后进行PID温度控制*/
    if (first_temperate)
    {
				/*(3.1)	PID计算PWM输出值*/
        PID_calc(&imu_temp_pid, temp, IMU_Temp_Set);
			/*(3.2)	如果输出值小于零，PWM没法儿小于零，设成零不加热*/
        if (imu_temp_pid.out < 0.0f)    imu_temp_pid.out = 0.0f;
        
				/*(3.3)	pid输出值赋给PWM变量并产生PWM波加热*/
        tempPWM = (uint16_t)imu_temp_pid.out;
        IMU_temp_PWM(tempPWM);
    }
    else
    {
       /*(2)	达到设定温度并持续200个运行周期，将积分项设置为最大功率的一半，加速收敛？？(这个迅速加热程序只执行一次，后面靠PID温控，所以连temp_constant_time都没清零)*/
        if (temp > IMU_Temp_Set)
        {
					/*(2.1)	要超过设定温度一段时间才能认为完全打到设定温度*/
            temp_constant_time++;
            if (temp_constant_time > 200)
            {
							/*(2.2)	快速升温第一次达到目标温度，上面那个if判断为真了，以后就执行PID温控程序*/
                first_temperate = 1;
							/*(2.3)	将积分项设置为最大功率的一半，加速收敛*/
                imu_temp_pid.Iout = MPU6500_TEMP_PWM_MAX / 2.0f;
            }
        }
				/*(1)	没有达到设定温度一直以最大功率加热,快速到目标温度附近*/
        IMU_temp_PWM(MPU6500_TEMP_PWM_MAX - 1);
    }
}

/**[2.3]
  * @brief          旋转陀螺仪,加速度计和磁力计,并计算零漂,因为设备有不同安装方式
  * @param[out]     gyro: 加上零漂和旋转
  * @param[out]     accel: 加上零漂和旋转
  * @param[out]     mag: 加上零漂和旋转
  * @param[in]      bmi088: 陀螺仪和加速度计数据
  * @param[in]      ist8310: 磁力计数据
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



/*****************************[3]	这些函数都没用到********************************************/
/**
  * @brief          计算陀螺仪零漂
  * @param[out]     gyro_offset:计算零漂
  * @param[in]      gyro:角速度数据
  * @param[out]     offset_time_count: 自动加1
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
  * @brief          校准陀螺仪
  * @param[out]     陀螺仪的比例因子，1.0f为默认值，不修改
  * @param[out]     陀螺仪的零漂，采集陀螺仪的静止的输出作为offset
  * @param[out]     陀螺仪的时刻，每次在gyro_offset调用会加1,
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
  * @brief          校准陀螺仪设置，将从flash或者其他地方传入校准值
  * @param[in]      陀螺仪的比例因子，1.0f为默认值，不修改
  * @param[in]      陀螺仪的零漂
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
  * @brief          获取四元数
  * @param[in]      none
  * @retval         INS_quat的指针
  */
const fp32 *get_INS_quat_point(void)
{
    return INS_quat;
}

/**
  * @brief          获取欧拉角, 0:yaw, 1:pitch, 2:roll 单位 rad
  * @param[in]      none
  * @retval         INS_angle的指针
  */
const fp32 *get_INS_angle_point(void)
{
    return INS_angle;
}


/**
  * @brief          获取角速度,0:x轴, 1:y轴, 2:roll轴 单位 rad/s
  * @param[in]      none
  * @retval         INS_gyro的指针
  */
extern const fp32 *get_gyro_data_point(void)
{
    //return INS_gyro;
		return bmi088_real_data.gyro;
}

/**
  * @brief          获取加速度,0:x轴, 1:y轴, 2:roll轴 单位 m/s2
  * @param[in]      none
  * @retval         INS_accel的指针
  */
extern const fp32 *get_accel_data_point(void)
{
    return INS_accel;
}

/**
  * @brief          获取加速度,0:x轴, 1:y轴, 2:roll轴 单位 ut
  * @param[in]      none
  * @retval         INS_mag的指针
  */
extern const fp32 *get_mag_data_point(void)
{
    return INS_mag;
}


/**
  * @brief          计算IMU零漂
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


