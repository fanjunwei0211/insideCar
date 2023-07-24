#include "move_task.h"
#include "main.h"
#include "chasis_task.h"

extern char probe_init_flag,probe_move_flag;
char forward_cnt = 0;
extern char start_flag;

void Move_task(void const * argument)
{
	osDelay(50);// 等待其他外设初始化完成
	while(probe_init_flag == 0)// 等待探头转向旋转初始化完成
	{
		osDelay(1);
	}
	while(start_flag == 0){osDelay(1);}
	while(forward_cnt<4)
	{
		insideCar_speed = 500;
		osDelay(4000);
		insideCar_speed = 0;
		probe_move_flag = 1;
		osDelay(4000);
		probe_move_flag = 0;
		forward_cnt++;
	}
	while(forward_cnt>0)
	{
		insideCar_speed = -500;
		osDelay(4000);
		insideCar_speed = 0;
		probe_move_flag = 1;
		osDelay(4000);
		probe_move_flag = 0;
		forward_cnt--;
	}
  for(;;)
	{
		
			
	}
}

	