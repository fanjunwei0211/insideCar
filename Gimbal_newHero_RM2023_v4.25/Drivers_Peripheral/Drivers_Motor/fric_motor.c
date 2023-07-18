#include "fric_motor.h"
#include "pid.h"
#include "motor_can.h"

/********************������Ϣ�Ϳ�����Ϣ********************/
motor_data_t 		fric_motor_data[2];				//��ŵ��������Ϣ
pid_type_def 		fric_motor_speed_pid[2];	//��ŵ��������Ϣ


/********************�������PID����********************/
fp32 fric_motor_speed_pid_Kp				=10.0f;
fp32 fric_motor_speed_pid_Ki				=0.005f;
fp32 fric_motor_speed_pid_Kd				=0.0f;
fp32 fric_motor_speed_pid_max_out		=16000.0f;
fp32 fric_motor_speed_pid_max_iout	=6000.0f;
fp32 fric_motor_speed_PID[3];


/**	[1]							Ħ���ֵ����ʼ��
  * brief         	��ʼ�������������Ϣ�Ϳ�����Ϣ
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


/**[2]						����Ŀ���ٶȼ���Ŀ�����
  * brief																
	* postscript																																	*/
extern int16_t motor_3508current_on_CAN2_setting[4];//ͳһ��ֵ��ͳһ����
void calculate_fric_motor_current_with_target_speed()
{
		fric_motor_data[0].target_current	=	PID_calc(	&fric_motor_speed_pid[0],	fric_motor_data[0].speed_rpm,	fric_motor_data[0].target_speed_rpm);
			
		fric_motor_data[1].target_current	=	PID_calc(	&fric_motor_speed_pid[1],	fric_motor_data[1].speed_rpm,	fric_motor_data[1].target_speed_rpm);	
}


