#include "yaw_motor.h"
#include "motor_can.h"
#include "pid.h"
#include "remote_control.h"
#include "BMI088driver.h"
#include <stdlib.h>
#include "math.h"
#include "Nmanifold_usart_task.h"
/*********************************************************������Ϣ�Ϳ�����Ϣ*********************************************************/
motor_data_t 		yaw_motor_data;				
pid_type_def		yaw_motor_INS_angle_pid;	
pid_type_def 		yaw_motor_INS_speed_pid;
pid_type_def 		yaw_motor_key_INS_speed_pid;
pid_type_def    yaw_motor_autoaim_INS_speed_pid;
pid_type_def    yaw_motor_autoaim_INS_angle_pid;
pid_type_def		yaw_motor_encoder_angle_pid;
pid_type_def 		yaw_motor_encoder_speed_pid;	

/*********************************************************�������PID����*********************************************************/
		/*****************ң��ģʽ*****************/
		/*yaw��			ң��ģʽ			�������ٶȻ�*/
		fp32 yaw_motor_INS_speed_pid_Kp				=10000.0f;
		fp32 yaw_motor_INS_speed_pid_Ki				=20.0f;
		fp32 yaw_motor_INS_speed_pid_Kd				=0.0f;
		fp32 yaw_motor_INS_speed_pid_max_out	=30000.0f;
		fp32 yaw_motor_INS_speed_pid_max_iout	=15000.0f;
		fp32 yaw_motor_INS_speed_PID[3]; 
		/*֮ǰ����(�µ�������)��(4000,20,0.30000,15000)(40000.0f��28.0f��0.0f��30000.0f��500.0f);*/

		/*yaw��			ң��ģʽ			������λ�û�*/
		fp32 yaw_motor_INS_angle_pid_Kp				=0.8;
		fp32 yaw_motor_INS_angle_pid_Ki				=0.0f;
		fp32 yaw_motor_INS_angle_pid_Kd				=2.0f;
		fp32 yaw_motor_INS_angle_pid_max_out	=10.0f;
		fp32 yaw_motor_INS_angle_pid_max_iout	=0.0f;
		fp32 yaw_motor_INS_angle_PID[3];
		/*֮ǰ������(0.05f��0.0f��2.7f��10.0f��0.0f);	(�µ�������)*/

		/************��̨�������ģʽ*****************/
		/*yaw��		��̨������ģʽ		������λ�û�*/
		fp32 yaw_motor_encoder_angle_pid_Kp				=0.2;
		fp32 yaw_motor_encoder_angle_pid_Ki				=0.0f;
		fp32 yaw_motor_encoder_angle_pid_Kd				=1.0f;
		fp32 yaw_motor_encoder_angle_pid_max_out	=200.0f;
		fp32 yaw_motor_encoder_angle_pid_max_iout	=0.0f;
		fp32 yaw_motor_encoder_angle_PID[3];
		/*֮ǰ������();(0.05,0,1,200,0);(0.05,0,2.7,10,0)	(�µ�������)*/

		/*yaw��		��̨������ģʽ		�������ٶȻ�*/
		fp32 yaw_motor_encoder_speed_pid_Kp				=600.0f;
		fp32 yaw_motor_encoder_speed_pid_Ki				=10.0f;
		fp32 yaw_motor_encoder_speed_pid_Kd				=0.0f;
		fp32 yaw_motor_encoder_speed_pid_max_out	=20000.0f;
		fp32 yaw_motor_encoder_speed_pid_max_iout	=10000.0f;
		fp32 yaw_motor_encoder_speed_PID[3]; 
		/*֮ǰ������(180,3,0,20000,10000);(350,40,0,20000,10000);(40000,28,0,30000,500)		(�µ�������)*/

		/*****************����ģʽ*****************/
		/*yaw��		����ģʽ		�������ٶȻ�*/
		fp32 yaw_motor_autoaim_INS_speed_pid_Kp				=22500.0f;
		fp32 yaw_motor_autoaim_INS_speed_pid_Ki				=80.0f;
		fp32 yaw_motor_autoaim_INS_speed_pid_Kd				=6800.0f;
		fp32 yaw_motor_autoaim_INS_speed_pid_max_out	=30000.0f;
		fp32 yaw_motor_autoaim_INS_speed_pid_max_iout	=10000.0f;
		fp32 yaw_motor_autoaim_INS_speed_PID[3]; 
		/*֮ǰ������(30000.0f��30.0f��0.0f��30000.0f��500.0f);	(�µ�������)*/
		
		/*yaw��		����ģʽ		������λ�û�*/
		fp32 yaw_motor_autoaim_INS_angle_pid_Kp				=0.4f;
		fp32 yaw_motor_autoaim_INS_angle_pid_Ki				=1.5f;
		fp32 yaw_motor_autoaim_INS_angle_pid_Kd				=5.0f;
		fp32 yaw_motor_autoaim_INS_angle_pid_max_out	=100.0f;
		fp32 yaw_motor_autoaim_INS_angle_pid_max_iout	=0.0f;
		fp32 yaw_motor_autoaim_INS_angle_PID[3];
		/*֮ǰ������(0.4,1.5,5,100,0);(1.2,2.0,3.0,100,0);(0.45,0.0,3.0,100,0)	(�µ�������)*/
		
		/*****************����΢��ģʽ*****************/
		/*yaw��			����΢��ģʽ			�������ٶȻ�*/
		fp32 yaw_motor_key_INS_speed_pid_Kp				=30000.0f;
		fp32 yaw_motor_key_INS_speed_pid_Ki				=30.0f;
		fp32 yaw_motor_key_INS_speed_pid_Kd				=0.0f;
		fp32 yaw_motor_key_INS_speed_pid_max_out	=30000.0f;
		fp32 yaw_motor_key_INS_speed_pid_max_iout	=10000.0f;
		fp32 yaw_motor_key_INS_speed_PID[3]; 
/*********************************************************YAW���Ƴ���*********************************************************/

/*****yaw�������ȱ���*****/
#define YAW_RC_SENS 0.01f//yaw��ң�ؿ��Ƶ�������
//#define YAW_MOUSE_SENS 0.2f//yaw�������Ƶ�������
float	YAW_MOUSE_SENS=0.06f;//yaw�������Ƶ�������
float YAW_MOUSE_AIM_ON_SENS=0.02f;//����֮�������Ƶ�������

/*****��̨������̵�6020ƫ��*****/
#define Gimbal_Follow_Chassis_Offset_Ecd	6920

/*****yaw��Ŀ���ģʽ*****/
#define Yaw_RC_Mode				0								//ң��ģʽ
#define Yaw_Autoaim_Mode	1								//����ģʽ
#define Yaw_Gimbal_Follow_Chassis_Mode 2	//���̸�����̨ģʽ
/*****���������ݱ���*****/
extern fp32 INS_angle_deg[3];//ŷ����(degree)
extern bmi088_real_data_t bmi088_real_data;

/*****yaw�������Ǳ���*****/
float yaw_INS_speed;								//yaw���������ٶ�
float yaw_INS_angle;								//yaw�������ǽǶ�

float yaw_target_INS_speed;					//yaw��������Ŀ���ٶ�
float yaw_target_INS_angle;					//yaw��������Ŀ��Ƕ�

float yaw_autoaim_target_INS_speed;	//yaw��������Ŀ���ٶ�
float yaw_autoaim_target_INS_angle;	//yaw��������Ŀ��Ƕ�

float yaw_encoder_speed;						//yaw��������ٶ�
float yaw_encoder_angle;						//yaw��������Ƕ�

float yaw_target_encoder_speed;			//yaw�������Ŀ���ٶ�
float yaw_target_encoder_angle;			//yaw�������Ŀ��Ƕ�

/*****yaw����ģʽ�ı�־����*****/
uint8_t yaw_mode_flag;//0��ʾң�أ�1��ʾ���飬2��ʾ��̨�������
uint8_t yaw_mode_flag_last;

/*****yaw�Ƿ��ڶ��ı�־����*****/
uint8_t yaw_moving_flag;//1��ʾyaw�ڶ���0��ʾyaw����
uint8_t yaw_moving_last_flag;

/*****yaw����΢������*****/
int16_t KEY_Z;
int16_t KEY_Z_last;
int16_t KEY_X;
int16_t KEY_X_last;

/*****���յ��Ӿ��������*****/
extern AutoAim_Data_Rx AutoAim_Data_Receive;

extern uint8_t aim_on_off_flag;//������־λ
/**[]					
  * brief					����yaw���											
	* postscript							*/
uint8_t Autoaim_UI_flag;
void Yaw_Motor_Control()
{
	if(rc_ctrl.rc.s[1]==1 || rc_ctrl.rc.s[1]==3)//�󲦸˲����м�����棬��̨��������
	{
		if(	(rc_ctrl.mouse.press_r)	&&	(AutoAim_Data_Receive.If_Aimed==1)	)//����Ҽ��������鲢�������⵽Ŀ�꣨���ϼ����Ҽ��������飩
//		if(	((rc_ctrl.mouse.press_r)||rc_ctrl.rc.s[1]==1)	&&	(AutoAim_Data_Receive.If_Aimed==1)	)//ֻ��ң�����������ʱ������Ҽ�������������󲦸˲���������  ���������⵽Ŀ�꣨���ϼ����Ҽ��������飩
		{Yaw_Motor_Control_in_Autoaim_Mode();Autoaim_UI_flag=1;}
		else//û������ûʶ��
		{Yaw_Motor_Control_in_RC_Mode();Autoaim_UI_flag=0;}
	}
	else if(rc_ctrl.rc.s[1]==2)
	{
		yaw_motor_data.target_current=0;
	}
	yaw_mode_flag_last=yaw_mode_flag;

}


/**	[1.1]							
  * brief         ����ģʽ�¿���yaw��
	* postscript										*/
static float yaw_last;
void Yaw_Motor_Control_in_Autoaim_Mode()
{
		yaw_mode_flag=Yaw_Autoaim_Mode;//���һ��yaw��ģʽ
	
		yaw_INS_angle=INS_angle_deg[0];//��ȡyaw�������ǽǶ�
		yaw_INS_speed=bmi088_real_data.gyro[2];//��ȡyaw���������ٶ�(�������֮ǰ�ƺ�û��仰����)
	
		yaw_autoaim_target_INS_angle=AutoAim_Data_Receive.Yaw;//��ȡ�Ӿ����ص�Ŀ��Ƕ�
		yaw_autoaim_target_INS_angle=0.8f*yaw_last+yaw_autoaim_target_INS_angle*0.2f;
		yaw_last=yaw_autoaim_target_INS_angle;//�ӵ�ͨ�˲�
	
	
		if(yaw_autoaim_target_INS_angle-yaw_INS_angle>180)yaw_INS_angle+=360;
		if(yaw_autoaim_target_INS_angle-yaw_INS_angle<-180)yaw_INS_angle-=360;//��ͷ���ӻ�
	
		calculate_yaw_motor_current_with_autoaim_target_INS_angle();//����ģʽλ�û�
}

/**	[1.2]							
  * brief         ң��ģʽ�¿���yaw��
	* postscript										*/
void  Yaw_Motor_Control_in_RC_Mode()
{
//		yaw_INS_speed =	bmi088_real_data.gyro[2];
//	
//	if(rc_ctrl.rc.s[0]==1)
//		yaw_target_INS_speed=10.0f;
//	if(rc_ctrl.rc.s[0]==3)
//		yaw_target_INS_speed=-10.0f;
//	if(rc_ctrl.rc.s[0]==2)
//		yaw_target_INS_speed=0.0f;
//	
//	calculate_yaw_motor_current_with_target_INS_speed();
	
	
		/*(0)	��ȡyaw����Ϣ������ģʽ����*/
	
			yaw_mode_flag=Yaw_RC_Mode;//���һ��yaw��ģʽ
				
			KEY_Z=((rc_ctrl.key.v & KEY_PRESSED_OFFSET_Z) >> 11);
			KEY_X=((rc_ctrl.key.v & KEY_PRESSED_OFFSET_X) >> 12);//��ȡ������Ϣ
	
			yaw_INS_speed =	bmi088_real_data.gyro[2];//��ȡyaw���������ٶ�
			yaw_INS_angle=INS_angle_deg[0];//��ȡyaw�������ǽǶ�

			if(yaw_mode_flag==0 && yaw_mode_flag_last!=0)//���������ģʽ�л���ң��ģʽ��ң��ģʽ��Ŀ��Ƕ�����ɵ�ǰ�Ƕ�
				yaw_target_INS_angle=INS_angle_deg[0];
			
			
		/*(1)	�ж�ͷ��û��ʵ���ڶ������Ϊ����״̬*/
			
			if(rc_ctrl.rc.ch[0]!=0 || rc_ctrl.mouse.x!=0)	yaw_moving_flag=1;
			if(rc_ctrl.rc.ch[0]==0 &&	rc_ctrl.mouse.x==0 && fabs(yaw_INS_speed)<0.5)	yaw_moving_flag=0;//����ң����ҡ�˺�yaw���ٶ� ��������ͬʱ�жϣ�����yaw�ٶ��жϿ��Է�ֹyaw����
		
		/*(2)	��ͷ����ͷ����������ͷ��������״̬�·ֱ�д����*/
			
			if(yaw_moving_flag==1)//ͷ����ʱ������Ŀ���ٶ�
			{
				if(aim_on_off_flag==0)//���û����������һ��������
					yaw_target_INS_speed=-YAW_RC_SENS*rc_ctrl.rc.ch[0]-YAW_MOUSE_SENS*rc_ctrl.mouse.x;
				else //��������ˣ���������ȵ���
					yaw_target_INS_speed=-YAW_RC_SENS*rc_ctrl.rc.ch[0]-YAW_MOUSE_AIM_ON_SENS*rc_ctrl.mouse.x;
				
				calculate_yaw_motor_current_with_target_INS_speed();
			}
			
			if(yaw_moving_last_flag==1 && yaw_moving_flag==0)//ͷ��ͣ����ʱ������Ŀ��Ƕ�
				yaw_target_INS_angle=INS_angle_deg[0];
			
			if(yaw_moving_flag==0)//	ͷ������ʱ��λ�û������Ƕ�
			{
				/*����1����������Ŀ��λ��*/
				if(KEY_Z_last==0	&&	KEY_Z!=0)yaw_target_INS_angle+=0.05f;
				if(KEY_X_last==0	&&	KEY_X!=0)yaw_target_INS_angle-=0.05f;
				
//				/*����2����������Ŀ���ٶ�*/
//				if(KEY_Z==1)//������°���
//				{
//					yaw_target_INS_speed=0.05f;
//					calculate_yaw_motor_current_with_target_key_INS_speed();
//				}
//				if(KEY_X==1)//������°���
//				{
//					yaw_target_INS_speed=-0.05f;
//					calculate_yaw_motor_current_with_target_key_INS_speed();
//				}					
//				if(	(KEY_Z_last==1	&&	KEY_Z==0)	||	(KEY_X_last==1	&&	KEY_X==0)	)yaw_target_INS_angle=yaw_INS_angle;
				
				if(KEY_Z==0 && KEY_X==0)//ң���������ȼ���ͣ������������ʱ����ֵ���
				{
					if(yaw_target_INS_angle-yaw_INS_angle>180)yaw_INS_angle+=360;
					if(yaw_target_INS_angle-yaw_INS_angle<-180)yaw_INS_angle-=360;//��ͷ���ӻ�
					calculate_yaw_motor_current_with_target_INS_angle();
				}

			}

			
			yaw_moving_last_flag=yaw_moving_flag;
			KEY_Z_last=KEY_Z;
			KEY_X_last=KEY_X;
}

/**	[1.3]							
  * brief         ��̨�������ģʽ�¿���yaw��
	* postscript										*/
void  Yaw_Motor_Control_in_Gimbal_Follow_Chassis_Mode()
{
		yaw_mode_flag=Yaw_Gimbal_Follow_Chassis_Mode;//���һ��yaw��ģʽ
	
		yaw_encoder_speed=yaw_motor_data.speed_rpm;//��ȡyaw�����������ٶ�
		yaw_encoder_angle=yaw_motor_data.ecd;//��ȡyaw�����������Ƕ�
	
		yaw_target_encoder_angle=Gimbal_Follow_Chassis_Offset_Ecd;//Ŀ��ֵΪ������ƫ�ýǶ�
	
		if(yaw_target_encoder_angle-yaw_encoder_angle>4096)yaw_encoder_angle+=8192;
		if(yaw_target_encoder_angle-yaw_encoder_angle<-4096)yaw_encoder_angle-=8192;//��ͷ���ӻ�
	
		calculate_yaw_motor_current_with_target_encoder_angle();//�����������λ�û�
}


/**[2.1]					
  * brief					ң��ģʽ�µ��������ٶȻ�											
	* postscript						*/
void calculate_yaw_motor_current_with_target_INS_speed()
{
		yaw_motor_data.target_current	=	PID_calc(	&yaw_motor_INS_speed_pid,	yaw_INS_speed	,yaw_target_INS_speed);//yaw����ID��1	
}

/**[2.2]					
  * brief					ң��ģʽ�µ�������λ�û�												
	* postscript							*/
void calculate_yaw_motor_current_with_target_INS_angle()
{
		yaw_target_INS_speed=PID_calc(	&yaw_motor_INS_angle_pid,	yaw_INS_angle,	yaw_target_INS_angle);

		yaw_motor_data.target_current=PID_calc(	&yaw_motor_INS_speed_pid, yaw_INS_speed, yaw_target_INS_speed);//yaw����ID��1
}

/**[2.3]					
  * brief					����ģʽ�µ��������ٶȻ�											
	* postscript					*/
void calculate_yaw_motor_current_with_autoaim_target_INS_speed()
{
		yaw_motor_data.target_current	=	PID_calc(	&yaw_motor_autoaim_INS_speed_pid,	yaw_INS_speed	,yaw_autoaim_target_INS_speed);//yaw����ID��1	
}

/**[2.4]					
  * brief					����ģʽ�µ�������λ�û�											
	* postscript							*/
void calculate_yaw_motor_current_with_autoaim_target_INS_angle()
{
		yaw_autoaim_target_INS_speed=PID_calc(	&yaw_motor_autoaim_INS_angle_pid,	yaw_INS_angle,	yaw_autoaim_target_INS_angle);

		yaw_motor_data.target_current=PID_calc(	&yaw_motor_autoaim_INS_speed_pid, yaw_INS_speed, yaw_autoaim_target_INS_speed);//yaw����ID��1
}

/**[2.5]					
  * brief					��̨�������ģʽ�µı������ٶȻ�											
	* postscript							*/
void calculate_yaw_motor_current_with_target_encoder_speed()
{
		yaw_motor_data.target_current=PID_calc(	&yaw_motor_encoder_speed_pid, yaw_encoder_speed, yaw_target_encoder_speed);//yaw����ID��1
}

/**[2.6]					
  * brief					��̨�������ģʽ�µı�����λ�û�										
	* postscript							*/
void calculate_yaw_motor_current_with_target_encoder_angle()
{
		yaw_target_encoder_speed=PID_calc(	&yaw_motor_encoder_angle_pid,	yaw_encoder_angle,	yaw_target_encoder_angle);

		yaw_motor_data.target_current=PID_calc(	&yaw_motor_encoder_speed_pid, yaw_encoder_speed, yaw_target_encoder_speed);//yaw����ID��1
}

/**[2.7]					
  * brief					����΢��ģʽ�µ��������ٶȻ�											
	* postscript		pidҪ������һ������Ȼֱ��������pid�ᳬ��			*/
void calculate_yaw_motor_current_with_target_key_INS_speed()
{
		yaw_motor_data.target_current	=	PID_calc(	&yaw_motor_key_INS_speed_pid,	yaw_INS_speed	,yaw_target_INS_speed);//yaw����ID��1	
}

/**	[3]							yaw������ʼ��
  * brief         	��ʼ�������������Ϣ�Ϳ�����Ϣ
	* postscript										*/
void yaw_motor_init(void)
{
	yaw_motor_data.speed_rpm=0;
	yaw_motor_data.give_current=0;
	
	yaw_motor_INS_angle_PID[0]	=	yaw_motor_INS_angle_pid_Kp;
	yaw_motor_INS_angle_PID[1]	=	yaw_motor_INS_angle_pid_Ki;
	yaw_motor_INS_angle_PID[2]	=	yaw_motor_INS_angle_pid_Kd;
	PID_init(	&yaw_motor_INS_angle_pid,	PID_POSITION,	yaw_motor_INS_angle_PID,	yaw_motor_INS_angle_pid_max_out,	yaw_motor_INS_angle_pid_max_iout);
	
	yaw_motor_INS_speed_PID[0]	=	yaw_motor_INS_speed_pid_Kp;
	yaw_motor_INS_speed_PID[1]	=	yaw_motor_INS_speed_pid_Ki;
	yaw_motor_INS_speed_PID[2]	=	yaw_motor_INS_speed_pid_Kd;
	PID_init(	&yaw_motor_INS_speed_pid,	PID_POSITION,	yaw_motor_INS_speed_PID,	yaw_motor_INS_speed_pid_max_out,	yaw_motor_INS_speed_pid_max_iout);	
	
	yaw_motor_key_INS_speed_PID[0]	=	yaw_motor_key_INS_speed_pid_Kp;
	yaw_motor_key_INS_speed_PID[1]	=	yaw_motor_key_INS_speed_pid_Ki;
	yaw_motor_key_INS_speed_PID[2]	=	yaw_motor_key_INS_speed_pid_Kd;
	PID_init(	&yaw_motor_key_INS_speed_pid,	PID_POSITION,	yaw_motor_key_INS_speed_PID,	yaw_motor_key_INS_speed_pid_max_out,	yaw_motor_key_INS_speed_pid_max_iout);	
	
	yaw_motor_autoaim_INS_angle_PID[0]	=	yaw_motor_autoaim_INS_angle_pid_Kp;
	yaw_motor_autoaim_INS_angle_PID[1]	=	yaw_motor_autoaim_INS_angle_pid_Ki;
	yaw_motor_autoaim_INS_angle_PID[2]	=	yaw_motor_autoaim_INS_angle_pid_Kd;
	PID_init(	&yaw_motor_autoaim_INS_angle_pid,	PID_POSITION,	yaw_motor_autoaim_INS_angle_PID,	yaw_motor_autoaim_INS_angle_pid_max_out,	yaw_motor_autoaim_INS_angle_pid_max_iout);
	
	yaw_motor_autoaim_INS_speed_PID[0]	=	yaw_motor_autoaim_INS_speed_pid_Kp;
	yaw_motor_autoaim_INS_speed_PID[1]	=	yaw_motor_autoaim_INS_speed_pid_Ki;
	yaw_motor_autoaim_INS_speed_PID[2]	=	yaw_motor_autoaim_INS_speed_pid_Kd;
	PID_init(	&yaw_motor_autoaim_INS_speed_pid,	PID_POSITION,	yaw_motor_autoaim_INS_speed_PID,	yaw_motor_autoaim_INS_speed_pid_max_out,	yaw_motor_autoaim_INS_speed_pid_max_iout);	

	yaw_motor_encoder_speed_PID[0]	=	yaw_motor_encoder_speed_pid_Kp;
	yaw_motor_encoder_speed_PID[1]	=	yaw_motor_encoder_speed_pid_Ki;
	yaw_motor_encoder_speed_PID[2]	=	yaw_motor_encoder_speed_pid_Kd;
	PID_init(	&yaw_motor_encoder_speed_pid,	PID_POSITION,	yaw_motor_encoder_speed_PID,	yaw_motor_encoder_speed_pid_max_out,	yaw_motor_encoder_speed_pid_max_iout);
	
	yaw_motor_encoder_angle_PID[0]	=	yaw_motor_encoder_angle_pid_Kp;
	yaw_motor_encoder_angle_PID[1]	=	yaw_motor_encoder_angle_pid_Ki;
	yaw_motor_encoder_angle_PID[2]	=	yaw_motor_encoder_angle_pid_Kd;
	PID_init(	&yaw_motor_encoder_angle_pid,	PID_POSITION,	yaw_motor_encoder_angle_PID,	yaw_motor_encoder_angle_pid_max_out,	yaw_motor_encoder_angle_pid_max_iout);
}





/*********************************************************���Գ���*********************************************************/
	//���Գ���1�������ٶȻ����Գ���
/*
	yaw_INS_speed =	bmi088_real_data.gyro[2];
	
	if(rc_ctrl.rc.s[0]==1)
		yaw_autoaim_target_INS_speed=5.0f;
	if(rc_ctrl.rc.s[0]==3)
		yaw_autoaim_target_INS_speed=-5.0f;
	if(rc_ctrl.rc.s[0]==2)
		yaw_autoaim_target_INS_speed=0.0f;
	
	calculate_yaw_motor_current_with_autoaim_target_INS_speed();
*/

	//���Գ���2������λ�û����Գ���
/*
	yaw_INS_angle=INS_angle_deg[0];
	yaw_INS_speed =	bmi088_real_data.gyro[2];
	if(rc_ctrl.rc.s[0]==1)
		yaw_autoaim_target_INS_angle=170.0f;
	if(rc_ctrl.rc.s[0]==3)
		yaw_autoaim_target_INS_angle=-170.0f;
	if(rc_ctrl.rc.s[0]==2)
		yaw_autoaim_target_INS_angle=0.0f;
	
	if(yaw_autoaim_target_INS_angle-yaw_INS_angle>180)yaw_INS_angle+=360;
	if(yaw_autoaim_target_INS_angle-yaw_INS_angle<-180)yaw_INS_angle-=360;//��ͷ���ӻ�
	calculate_yaw_motor_current_with_autoaim_target_INS_angle();
*/

		//���Գ���3����������Գ���
/*
	yaw_INS_angle=INS_angle_deg[0];
	yaw_INS_speed =	bmi088_real_data.gyro[2];
	
	if(AutoAim_Data_Receive.If_Aimed==1)//���ʶ��װ�װ�
		yaw_autoaim_target_INS_angle=AutoAim_Data_Receive.Yaw;
	else//ûʶ��װ�װ�Ͷ���ԭ��
		yaw_autoaim_target_INS_angle=yaw_INS_angle;
	
	if(yaw_autoaim_target_INS_angle-yaw_INS_angle>180)yaw_INS_angle+=360;
	if(yaw_autoaim_target_INS_angle-yaw_INS_angle<-180)yaw_INS_angle-=360;//��ͷ���ӻ�
	calculate_yaw_motor_current_with_autoaim_target_INS_angle();
*/

		//���Գ���4��һֱ��������Գ���
/*
	yaw_INS_angle=INS_angle_deg[0];
	yaw_INS_speed =	bmi088_real_data.gyro[2];
	
	if(AutoAim_Data_Receive.If_Aimed==1)//���ʶ��װ�װ�
	{	
		yaw_autoaim_target_INS_angle=AutoAim_Data_Receive.Yaw;
	
		if(yaw_autoaim_target_INS_angle-yaw_INS_angle>180)yaw_INS_angle+=360;
		if(yaw_autoaim_target_INS_angle-yaw_INS_angle<-180)yaw_INS_angle-=360;//��ͷ���ӻ�
		
		calculate_yaw_motor_current_with_autoaim_target_INS_angle();
	}
		else
			yaw_motor_data.target_current=0;
*/

		//���Գ���5��yaw���������pitch������pid
/*
			yaw_motor_data.target_current=0;	
*/

		//���Գ���6����̨��������ٶȻ����Գ���
/*
		yaw_encoder_speed =	yaw_motor_data.speed_rpm;
		
		if(rc_ctrl.rc.s[0]==1)
			yaw_target_encoder_speed=35.0f;
		if(rc_ctrl.rc.s[0]==3)
			yaw_target_encoder_speed=-35.0f;
		if(rc_ctrl.rc.s[0]==2)
			yaw_target_encoder_speed=0.0f;
		
		calculate_yaw_motor_current_with_target_encoder_speed();
*/
