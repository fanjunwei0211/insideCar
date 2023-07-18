#include "comm_task.h"
#include "FreeRTOS.h"
#include "task.h"

#include "motor_can.h"
#include "can_transmit.h"


extern uint8_t ChassisMode;
extern int16_t chassis_target_forward_speed_x,chassis_target_traverse_speed_y,chassis_target_rotation_speed_z;
extern uint8_t Autoaim_UI_flag;
extern uint16_t xiaotuoluo_UI_flag;
extern uint16_t fric_UI_flag;
extern float INS_roll_for_UI;
extern uint8_t down_dial_on_or_off_flag;
extern fp32 INS_angle_deg[3];
void comm_task(void const * argument)
{
	while(1)
	{
		/*	将目标移动速度发给底盘A板*/
		send_chassis_target_speed_by_CAN1(ChassisMode,chassis_target_forward_speed_x,chassis_target_traverse_speed_y,chassis_target_rotation_speed_z);//调自瞄先把底盘废掉
		
		vTaskDelay(5);
		
		send_UI_data_for_Chassis_by_CAN1(Autoaim_UI_flag,xiaotuoluo_UI_flag,fric_UI_flag,INS_roll_for_UI,down_dial_on_or_off_flag);
		vTaskDelay(5);
		
		send_UI_pitch_data_for_Chassis_by_CAN1(INS_angle_deg[2]);
			vTaskDelay(5);
	}

}
