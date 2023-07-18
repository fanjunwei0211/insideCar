#include "dial_motor.h"
#include "motor_can.h"
#include "pid.h"

/********************������Ϣ�Ϳ�����Ϣ********************/
motor_data_t dial_motor_data[2];
pid_type_def dial_motor_speed_pid[2];
pid_type_def dial_motor_angle_pid[2];	//��ŵ��������Ϣ

/********************�������PID����********************/
fp32 up_dial_motor_speed_pid_Kp=3.0f;
fp32 up_dial_motor_speed_pid_Ki=0.01f;
fp32 up_dial_motor_speed_pid_Kd=0.0f;
fp32 up_dial_motor_speed_pid_max_out=16000.0f;
fp32 up_dial_motor_speed_pid_max_iout=6000.0f;
fp32 up_dial_motor_speed_PID[3];

fp32 up_dial_motor_angle_pid_Kp				=0.2f;
fp32 up_dial_motor_angle_pid_Ki				=0.0f;
fp32 up_dial_motor_angle_pid_Kd				=0.2f;
fp32 up_dial_motor_angle_pid_max_out	=	10000.0f;
fp32 up_dial_motor_angle_pid_max_iout	=10000.0f;
fp32 up_dial_motor_angle_PID[3];//λ�û�


fp32 down_dial_motor_speed_pid_Kp=1.5f;//1.5
fp32 down_dial_motor_speed_pid_Ki=0.012f;//0.012
fp32 down_dial_motor_speed_pid_Kd=0.0f;
fp32 down_dial_motor_speed_pid_max_out=16000.0f;
fp32 down_dial_motor_speed_pid_max_iout=7200.0f;//���²����̵Ķ�ת����ֱ����أ�ԭ��6000������Pout����ת������8000���Ƕ��߷��ӻᷢ��
fp32 down_dial_motor_speed_PID[3];

fp32 down_dial_motor_angle_pid_Kp				=0.3f;
fp32 down_dial_motor_angle_pid_Ki				=0.0f;
fp32 down_dial_motor_angle_pid_Kd				=2.0f;
fp32 down_dial_motor_angle_pid_max_out	=	10000.0f;
fp32 down_dial_motor_angle_pid_max_iout	=0.0f;
fp32 down_dial_motor_angle_PID[3];//λ�û�

/**	[1]							�����̵����ʼ��
  * brief         	��ʼ�������������Ϣ�Ϳ�����Ϣ
	* postscript										*/
void dial_motor_init()
{	
		dial_motor_data[0].speed_rpm=0;
		dial_motor_data[0].give_current=0;
	
		up_dial_motor_speed_PID[0]=up_dial_motor_speed_pid_Kp;
		up_dial_motor_speed_PID[1]=up_dial_motor_speed_pid_Ki;
		up_dial_motor_speed_PID[2]=up_dial_motor_speed_pid_Kd;
		PID_init(	&dial_motor_speed_pid[0],	PID_POSITION,	up_dial_motor_speed_PID,	up_dial_motor_speed_pid_max_out,	up_dial_motor_speed_pid_max_iout	);
		
		up_dial_motor_angle_PID[0]	=	up_dial_motor_angle_pid_Kp;
		up_dial_motor_angle_PID[1]	=	up_dial_motor_angle_pid_Ki;
		up_dial_motor_angle_PID[2]	=	up_dial_motor_angle_pid_Kd;
		PID_init(	&dial_motor_angle_pid[0],	PID_POSITION,	up_dial_motor_angle_PID,	up_dial_motor_angle_pid_max_out,	up_dial_motor_angle_pid_max_iout);
	
	
	
		dial_motor_data[1].speed_rpm=0;
		dial_motor_data[1].give_current=0;
	
		down_dial_motor_speed_PID[0]=down_dial_motor_speed_pid_Kp;
		down_dial_motor_speed_PID[1]=down_dial_motor_speed_pid_Ki;
		down_dial_motor_speed_PID[2]=down_dial_motor_speed_pid_Kd;
		PID_init(	&dial_motor_speed_pid[1],	PID_POSITION,	down_dial_motor_speed_PID,	down_dial_motor_speed_pid_max_out,	down_dial_motor_speed_pid_max_iout	);
		
		down_dial_motor_angle_PID[0]	=	down_dial_motor_angle_pid_Kp;
		down_dial_motor_angle_PID[1]	=	down_dial_motor_angle_pid_Ki;
		down_dial_motor_angle_PID[2]	=	down_dial_motor_angle_pid_Kd;
		PID_init(	&dial_motor_angle_pid[1],	PID_POSITION,	down_dial_motor_angle_PID,	down_dial_motor_angle_pid_max_out,	down_dial_motor_angle_pid_max_iout);

}

/**[2]						����Ŀ���ٶȼ���Ŀ�����
  * brief																
	* postscript							*/
void calculate_up_dial_motor_current_with_target_speed()
{
		dial_motor_data[0].target_current	=	PID_calc(	&dial_motor_speed_pid[0],	dial_motor_data[0].speed_rpm,	dial_motor_data[0].target_speed_rpm);		
}

/**[2]						����Ŀ���ٶȼ���Ŀ�����
  * brief																
	* postscript							*/
void calculate_down_dial_motor_current_with_target_speed()
{
		dial_motor_data[1].target_current	=	PID_calc(	&dial_motor_speed_pid[1],	dial_motor_data[1].speed_rpm,	dial_motor_data[1].target_speed_rpm);
}


/**[2.3]					����Ŀ��Ƕȼ���Ŀ���ٶ��ټ���Ŀ�����
  * brief																
	* postscript							*/
void calculate_up_dial_motor_current_with_target_total_angle()
{
		dial_motor_data[0].target_speed_rpm=PID_calc(	&dial_motor_angle_pid[0],	dial_motor_data[0].total_ecd, dial_motor_data[0].target_total_ecd);

		dial_motor_data[0].target_current= PID_calc(	&dial_motor_speed_pid[0],	dial_motor_data[0].speed_rpm,	dial_motor_data[0].target_speed_rpm);//yaw����ID��1
}

/**[2.3]					����Ŀ��Ƕȼ���Ŀ���ٶ��ټ���Ŀ�����
  * brief																
	* postscript							*/
void calculate_down_dial_motor_current_with_target_total_angle()
{
		dial_motor_data[1].target_speed_rpm=PID_calc(	&dial_motor_angle_pid[1],	dial_motor_data[1].total_ecd, dial_motor_data[1].target_total_ecd);

		dial_motor_data[1].target_current= PID_calc(	&dial_motor_speed_pid[1],	dial_motor_data[1].speed_rpm,	dial_motor_data[1].target_speed_rpm);//yaw����ID��1
}
