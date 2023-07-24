#include "probeWave_task.h"
#include "main.h"
#include "bsp_can.h"
#include "chasis_task.h"


int16_t probe_speed = 0;     //内爬探头旋转速度   范围-16000~+16000
char probe_move_flag = 0,probe_init_flag = 0;    //内爬探头旋转开关   0-->关    1-->开
int32_t angle_probe_min = 0,
				angle_probe_max = 0;    //内爬小车旋转探头的转动角度

void ProbeWave_task(void const * argument)
{
	probe_position_init();
  for(;;)
  {
		while(moto_chassis[4].total_angle > angle_probe_min)
		{
			if(probe_move_flag == 1)
				probe_speed = -300;
			else break;
			osDelay(1);
		}
		osDelay(1);
		
		while(moto_chassis[4].total_angle < angle_probe_max)
		{
			if(probe_move_flag == 1)
				probe_speed = 300;
			else break;
			osDelay(1);
		}
		if(probe_move_flag == 0)
		{
			probe_speed = 0;
		}
    osDelay(1);
  }
}

void probe_position_init()
{
	probe_speed = 200;
	osDelay(200);
	while(1)
	{
		if(moto_chassis[4].speed_rpm <= probe_speed/5)
		{
			probe_speed  = 0;
			angle_probe_max = moto_chassis[4].total_angle - 1000;
			angle_probe_min = angle_probe_max - 19*8192/2 ;
			probe_init_flag = 1;
			break;
		}
		osDelay(1);
	}
}
