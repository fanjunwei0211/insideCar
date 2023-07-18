#include "fric_motor.h"
#include "pid.h"
#include "motor_can.h"

/********************数据信息和控制信息********************/
motor_data_t 		fric_motor_data[2];				//存放电机数据信息
pid_type_def 		fric_motor_speed_pid[2];	//存放电机控制信息


/********************电机控制PID参数********************/
fp32 fric_motor_speed_pid_Kp				=10.0f;
fp32 fric_motor_speed_pid_Ki				=0.005f;
fp32 fric_motor_speed_pid_Kd				=0.0f;
fp32 fric_motor_speed_pid_max_out		=16000.0f;
fp32 fric_motor_speed_pid_max_iout	=6000.0f;
fp32 fric_motor_speed_PID[3];


/**	[1]							摩擦轮电机初始化
  * brief         	初始化电机的数据信息和控制信息
  * postscript										*/
void fric_motor_init(void)
{
	fric_motor_data[0].speed_rpm		=0;
	fric_motor_data[0].give_current	=0;
	fric_motor_data[1].speed_rpm		=0;
	fric_motor_data[1].give_current	=0;
	
	fric_motor_speed_PID[0]	=	fric_motor_speed_pid_Kp;
	fric_motor_speed_PID[1]	=	fric_motor_speed_pid_Ki;
	fric_motor_speed_PID[2]	=	fric_motor_speed_pid_Kd;
	PID_init(	&fric_motor_speed_pid[0],	PID_DELTA,	fric_motor_speed_PID,	fric_motor_speed_pid_max_out,	fric_motor_speed_pid_max_iout);
	PID_init(	&fric_motor_speed_pid[1],	PID_DELTA,	fric_motor_speed_PID,	fric_motor_speed_pid_max_out,	fric_motor_speed_pid_max_iout);
}


/**[2]						根据目标速度计算目标电流
  * brief																
	* postscript																																	*/
extern int16_t motor_3508current_on_CAN2_setting[4];//统一赋值，统一发送
void calculate_fric_motor_current_with_target_speed()
{
		fric_motor_data[0].target_current	=	PID_calc(	&fric_motor_speed_pid[0],	fric_motor_data[0].speed_rpm,	fric_motor_data[0].target_speed_rpm);
			
		fric_motor_data[1].target_current	=	PID_calc(	&fric_motor_speed_pid[1],	fric_motor_data[1].speed_rpm,	fric_motor_data[1].target_speed_rpm);	
}


