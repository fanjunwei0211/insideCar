#include "servo.h"
#include "bsp_pwm.h"

#include "FreeRTOS.h"
#include "task.h"//���Ӳ���ʶvTaskDelay

#include "remote_control.h"
#include "fric_motor.h"
#include "motor_can.h"

/**[1]						
  * brief				    ��ʼ�����Ҷ���������PWM
	* postscript		  ��ʱ��8��ʼ�������Σ����ܲ��ܰ�һ�����ɶ�ʱ��1	*/
void servo_init(void)
{																
		PWM_init(&htim8,TIM_CHANNEL_2);//����仹���Ҷ���?
		PWM_init(&htim8,TIM_CHANNEL_3);//����仹���Ҷ���?
}

/**[2]						
  * brief				    �ö��������
	* postscript		  �ȴ�������д�����滹Ҫд��ϸ�µĶ������	*/
extern motor_data_t 		fric_motor_data[2];				//��ŵ��������Ϣ
void servo_ear(void)
{

}

