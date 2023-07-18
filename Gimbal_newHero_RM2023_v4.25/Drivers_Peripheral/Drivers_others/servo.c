#include "servo.h"
#include "bsp_pwm.h"

#include "FreeRTOS.h"
#include "task.h"//不加不认识vTaskDelay

#include "remote_control.h"
#include "fric_motor.h"
#include "motor_can.h"

/**[1]						
  * brief				    初始化左右耳朵舵机连的PWM
	* postscript		  定时器8初始化了两次，看能不能把一个换成定时器1	*/
void servo_init(void)
{																
		PWM_init(&htim8,TIM_CHANNEL_2);//左耳朵还是右耳朵?
		PWM_init(&htim8,TIM_CHANNEL_3);//左耳朵还是右耳朵?
}

/**[2]						
  * brief				    让耳朵摆起来
	* postscript		  先大致这样写，后面还要写个细致的耳朵程序	*/
extern motor_data_t 		fric_motor_data[2];				//存放电机数据信息
void servo_ear(void)
{

}

