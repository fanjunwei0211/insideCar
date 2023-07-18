#include "bsp_pwm.h"


/**[1]						
  * brief				    初始化定时器和定时器通道PWM
	* param[in]				定时器TIMx
	* param[in]				定时器通道
	* postscript		  定时器和通道PWM封装成一个函数一块初始化的话，可能出现定时器多次初始化的情况，比如初始化TIM1的通道1和通道2，需要调用两次		*/
void PWM_init(TIM_HandleTypeDef *htim, uint32_t Channel)
{
	HAL_TIM_Base_Start(htim);//开启定时器

	HAL_TIM_PWM_Start(htim,Channel);//开启PWM输出
}


void imu_pwm_init()
{
	PWM_init(&htim10, TIM_CHANNEL_1);
}
/**[2]						
  * brief				    控制imu的加热电阻的pwm
	* param[in]				重装载值
	* postscript		  定时器10的通道1		*/
extern TIM_HandleTypeDef htim10;
void imu_pwm_set(uint16_t pwm)
{
    __HAL_TIM_SetCompare(&htim10, TIM_CHANNEL_1, pwm);
}
