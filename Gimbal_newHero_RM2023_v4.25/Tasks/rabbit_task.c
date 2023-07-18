#include "rabbit_task.h"
#include "tim.h"
#include "FreeRTOS.h"
#include "task.h"
#include "servo.h"
void rabbit_task(void const * argument)
{
	while(1)
	{
//		servo_ear();
	vTaskDelay(2);
	}
}
