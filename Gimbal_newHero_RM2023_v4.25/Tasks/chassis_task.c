#include "chassis_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "remote_control.h"
#include "can_transmit.h"
#include "Referee.h"

#define RC_SW_UP                ((uint16_t)1)
#define RC_SW_MID               ((uint16_t)3)
#define RC_SW_DOWN              ((uint16_t)2)

///*70W�Ĳ���*/
//#define CHASSIS_RC_MOVE_SENS 11.0f//ԭ��3.0f��11.0f
//#define CHASSIS_KEY_MOVE_SENS 5500.0f//4000.0f
//#define CHASSIS_KEY_hengyi_MOVE_SENS 4000.0f
//#define CHASSIS_RC_ROTATE_SENS 11.0f//ԭ��3.0f;11.0f
//#define CHASSIS_KEY_ROTATE_SENS 4000.0f//6000.0f
/*70W�Ĳ���*/
float CHASSIS_RC_MOVE_SENS=11.0f;//ԭ��3.0f��11.0f
float CHASSIS_KEY_MOVE_SENS=5500.0f;//4000.0f
float CHASSIS_KEY_hengyi_MOVE_SENS=4000.0f;
float CHASSIS_RC_ROTATE_SENS=11.0f;//ԭ��3.0f;11.0f
float CHASSIS_KEY_ROTATE_SENS=4000.0f;//6000.0f
extern RC_ctrl_t rc_ctrl;

int16_t chassis_target_forward_speed_x,chassis_target_traverse_speed_y,chassis_target_rotation_speed_z;
uint8_t ChassisMode;
extern ext_game_robot_state_t Game_Robot_State;
uint16_t xiaotuoluo_UI_flag;
void chassis_task(void const * argument)
{
	while(1)
	{
		/*(1)	����ң�������õ����˶�ģʽ��Ŀ���ƶ��ٶ�*/
		switch(rc_ctrl.rc.s[1])
		{
			case RC_SW_DOWN: ChassisMode = 2;break;
			case RC_SW_MID:  ChassisMode = 3;break;
			case RC_SW_UP:   ChassisMode = 1;break;
			default:         ChassisMode = 0;break;
		}
		
		if(Game_Robot_State.chassis_power_limit==70)
		{
			CHASSIS_KEY_MOVE_SENS=5500.0f;
			CHASSIS_KEY_hengyi_MOVE_SENS=3500.0f;
			CHASSIS_KEY_ROTATE_SENS=3600.0f;//4000
		}
		if(Game_Robot_State.chassis_power_limit==100 || Game_Robot_State.chassis_power_limit==90)//������100W��������90W�Ĺ��ʣ���Ϊ���������費��90W�������˸ľ���
		{
			CHASSIS_KEY_MOVE_SENS=6600.0f;
			CHASSIS_KEY_hengyi_MOVE_SENS=4000.0f;
			CHASSIS_KEY_ROTATE_SENS=4300.0f;//4800
		}
		if(Game_Robot_State.chassis_power_limit==120)
		{
			CHASSIS_KEY_MOVE_SENS=8000.0f;
			CHASSIS_KEY_hengyi_MOVE_SENS=4500.0f;
			CHASSIS_KEY_ROTATE_SENS=5000.0f;//5500
		}
		
		
		chassis_target_forward_speed_x	=	CHASSIS_RC_MOVE_SENS*rc_ctrl.rc.ch[3]		- 	((rc_ctrl.key.v & KEY_PRESSED_OFFSET_S) >> 1) 		* CHASSIS_KEY_MOVE_SENS 		+ 	((rc_ctrl.key.v & KEY_PRESSED_OFFSET_W) >> 0) * CHASSIS_KEY_MOVE_SENS;
		chassis_target_traverse_speed_y	= CHASSIS_RC_MOVE_SENS*rc_ctrl.rc.ch[2]		- 	((rc_ctrl.key.v & KEY_PRESSED_OFFSET_A) >> 2) 		* CHASSIS_KEY_hengyi_MOVE_SENS 		+ 	((rc_ctrl.key.v & KEY_PRESSED_OFFSET_D) >> 3) * CHASSIS_KEY_hengyi_MOVE_SENS;
		chassis_target_rotation_speed_z	=	CHASSIS_RC_ROTATE_SENS*rc_ctrl.rc.ch[4]	+		((rc_ctrl.key.v & KEY_PRESSED_OFFSET_SHIFT) >> 4) * CHASSIS_KEY_ROTATE_SENS;
		if(((rc_ctrl.key.v & KEY_PRESSED_OFFSET_V) >> 14)==1)
			{
				chassis_target_rotation_speed_z*=1.4;//��V��߳��٣�����
				chassis_target_forward_speed_x*=1.4;
				chassis_target_traverse_speed_y*=1.4;
			}
			
			if(chassis_target_rotation_speed_z!=0)xiaotuoluo_UI_flag=1;
			else xiaotuoluo_UI_flag=0;
			
		vTaskDelay(2);
	}

		
}

