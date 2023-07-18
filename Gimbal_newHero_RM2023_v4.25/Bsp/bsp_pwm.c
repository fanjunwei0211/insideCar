#include "bsp_pwm.h"


/**[1]						
  * brief				    ��ʼ����ʱ���Ͷ�ʱ��ͨ��PWM
	* param[in]				��ʱ��TIMx
	* param[in]				��ʱ��ͨ��
	* postscript		  ��ʱ����ͨ��PWM��װ��һ������һ���ʼ���Ļ������ܳ��ֶ�ʱ����γ�ʼ��������������ʼ��TIM1��ͨ��1��ͨ��2����Ҫ��������		*/
void PWM_init(TIM_HandleTypeDef *htim, uint32_t Channel)
{
	HAL_TIM_Base_Start(htim);//������ʱ��

	HAL_TIM_PWM_Start(htim,Channel);//����PWM���
}


void imu_pwm_init()
{
	PWM_init(&htim10, TIM_CHANNEL_1);
}
/**[2]						
  * brief				    ����imu�ļ��ȵ����pwm
	* param[in]				��װ��ֵ
	* postscript		  ��ʱ��10��ͨ��1		*/
extern TIM_HandleTypeDef htim10;
void imu_pwm_set(uint16_t pwm)
{
    __HAL_TIM_SetCompare(&htim10, TIM_CHANNEL_1, pwm);
}
