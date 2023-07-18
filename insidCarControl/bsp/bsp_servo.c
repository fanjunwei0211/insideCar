#include "bsp_servo.h"
#include "main.h"


extern TIM_HandleTypeDef htim1,htim8;
void servo_init()
{
	HAL_TIM_Base_Start(&htim1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);  
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
//	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
//	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
//	HAL_TIM_Base_Start(&htim8);
//	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
//	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
//	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);

}
void servo_test(void)
{
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 500);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 500);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 500);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 500);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 500);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 500);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, 500);
	HAL_Delay(2000);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 2000);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 2000);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 2000);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 2000);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 2000);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 2000);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, 2000);
	HAL_Delay(2000);
}

void servo_set(int id, int pwm)
{
	int pwmset=pwm;
	if(pwm>2500) pwmset = 2500;
	else if(pwm<500) pwmset = 500;
	switch(id){
    case 1:
       __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, pwmset);
       break;
    case 2:
       __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, pwmset);
       break; 
		case 3:
       __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, pwmset);
       break; 
		case 4:
       __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, pwmset);
       break; 
		case 5:
       __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmset);
       break; 
		case 6:
       __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwmset);
       break; 
		case 7:
       __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, pwmset);
       break; 
    default : 
       break;
	}
}
	
