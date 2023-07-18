/**
  ****************************(C) COPYRIGHT 2023 DuanZheng****************************
  * @file       pitch_motor.c/h
  * @brief      �������˹���ܿƼ����޹�˾���죬ף�ϰ��������
	*							EC-A4310-P2-36������Ҽ��4310����������ص�
  *             һ��Ӣ����̨�õĵ��Ϊ10���ٱȣ�3nmŤ�أ�С�����Ť��
  *             �����ŷ�����ģʽ�ųƼ�������۲���ƣ��ô�����PID���õ���
  * @note       �ٶȻ���Kp��Ϊ�Ŷ�����ϵ����Ki��Ϊ�����۲����棿��������˵�������ģ�ע�ͺ�˵������˵�Ĳ�һ������
  *             λ�û���Kp��Ϊ��������ϵ����Kd��Ϊ����ϵ��
	*							���⻹�����¶ȱ��������ر�����һ�ѱ���
  *        			��ͨ�����ò��������ף�������λ������
  * @history
  *  Version    Date            Author          Modification
  *  V1.0		    Feb-04-2023     Duan Zheng              1. done
  *	 V2.0				Apr-19-2023			Duan Zheng							2. done
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2023 DuanZheng****************************
  */

#include "pitch_motor.h"
#include "remote_control.h"
#include "motor_can.h"
#include "pid.h"
#include "vofa.h"
#include "Nmanifold_usart_task.h"
#include "stdlib.h"

#define MOTOR_ID 1						//���ID
#define MOTOR4310_CAN hcan2		//���ͨ��can��
motor4310_data_t motor4310_data[8]; //�����Ǹ����յ�����ݵĺ���Ҫ��

/*********************************************************������Ϣ�Ϳ�����Ϣ*********************************************************/
motor_data_t 		pitch_motor_data;						//��ŵ��������Ϣ
pid_type_def		pitch_motor_INS_angle_pid;	//��ŵ��������Ϣ
pid_type_def		pitch_motor_INS_speed_pid;
pid_type_def		pitch_motor_mouse_INS_angle_pid;	
pid_type_def		pitch_motor_mouse_INS_speed_pid;	
pid_type_def		pitch_motor_autoaim_INS_angle_pid;	
pid_type_def		pitch_motor_autoaim_INS_speed_pid;	
/*********************************************************�������PID����*********************************************************/
		/*****************ң��ģʽ*****************/
		/*pitch��			ң��ģʽ			�������ٶȻ�*/
		fp32 pitch_motor_INS_speed_pid_Kp				=1700.0f;
		fp32 pitch_motor_INS_speed_pid_Ki				=60.0f;
		fp32 pitch_motor_INS_speed_pid_Kd				=0.0f;
		fp32 pitch_motor_INS_speed_pid_max_out	=1000.0f;
		fp32 pitch_motor_INS_speed_pid_max_iout	=600.0f;
		fp32 pitch_motor_INS_speed_PID[3];

		/*pitch��			ң��ģʽ			������λ�û�*/
		fp32 pitch_motor_INS_angle_pid_Kp				=0.13f;//0.2f;
		fp32 pitch_motor_INS_angle_pid_Ki				=0.001f;//0.0f;
		fp32 pitch_motor_INS_angle_pid_Kd				=0.03f;//0.0f;
		fp32 pitch_motor_INS_angle_pid_max_out	=10.0f;//10.0f;
		fp32 pitch_motor_INS_angle_pid_max_iout	=2.0f;//0.0f;
		fp32 pitch_motor_INS_angle_PID[3];

		/*****************����ģʽ*****************/
		/*pitch��			����ģʽ			�������ٶȻ�*/
		fp32 pitch_motor_autoaim_INS_speed_pid_Kp				=1700.0f;
		fp32 pitch_motor_autoaim_INS_speed_pid_Ki				=60.0f;
		fp32 pitch_motor_autoaim_INS_speed_pid_Kd				=0.0f;
		fp32 pitch_motor_autoaim_INS_speed_pid_max_out	=1000.0f;
		fp32 pitch_motor_autoaim_INS_speed_pid_max_iout	=600.0f;
		fp32 pitch_motor_autoaim_INS_speed_PID[3];
		
		/*pitch��			����ģʽ			������λ�û�*/
		fp32 pitch_motor_autoaim_INS_angle_pid_Kp				=0.20f;
		fp32 pitch_motor_autoaim_INS_angle_pid_Ki				=0.0f;
		fp32 pitch_motor_autoaim_INS_angle_pid_Kd				=0.0f;
		fp32 pitch_motor_autoaim_INS_angle_pid_max_out	=10.0f;
		fp32 pitch_motor_autoaim_INS_angle_pid_max_iout	=0.0f;
		fp32 pitch_motor_autoaim_INS_angle_PID[3];

		/*****************���ģʽ*****************/
		/*pitch��			���ģʽ			�������ٶȻ�*/
		fp32 pitch_motor_mouse_INS_speed_pid_Kp				=1700.0f;
		fp32 pitch_motor_mouse_INS_speed_pid_Ki				=60.0f;
		fp32 pitch_motor_mouse_INS_speed_pid_Kd				=0.0f;
		fp32 pitch_motor_mouse_INS_speed_pid_max_out	=1000.0f;
		fp32 pitch_motor_mouse_INS_speed_pid_max_iout	=600.0f;
		fp32 pitch_motor_mouse_INS_speed_PID[3];

		/*pitch��			���ģʽ			������λ�û�*/
		fp32 pitch_motor_mouse_INS_angle_pid_Kp				=0.20f;
		fp32 pitch_motor_mouse_INS_angle_pid_Ki				=0.00f;
		fp32 pitch_motor_mouse_INS_angle_pid_Kd				=0.00f;
		fp32 pitch_motor_mouse_INS_angle_pid_max_out	=10.0f;
		fp32 pitch_motor_mouse_INS_angle_pid_max_iout	=0.0f;
		fp32 pitch_motor_mouse_INS_angle_PID[3];
/*********************************************************PITCH���Ƴ���*********************************************************/
/*****pitch��λ����*****/
#define PITCH_INS_MAX 2.5f		//�����λ���������Ƕ�7,-35
#define PITCH_INS_MIN -30.0f	//�����λ��С�������Ƕ�
#define INS_BUFFER 2.0f				//�����λ�Ƕȸ����Ļ�����

/*****pitch�����ȱ���*****/
//#define PITCH_RC_SENS 50.0f		//ң�����ٶ�������ˮƽ	
float PITCH_RC_SENS=3.0f;		//ң�����ٶ�������ˮƽ	
//#define PITCH_MOUSE_SENS 0.1f	//����ٶ�������
float PITCH_MOUSE_SENS=0.035f;	//����ٶ�������
float PITCH_MOUSE_AIM_ON_SENS=0.015f;//����֮�����������

/*****���������ݱ���*****/
extern fp32 INS_angle_deg[3];//ŷ����(degree)

/*****pitch�������Ǳ���*****/
float pitch_INS_angle;									//pitch�������ǽǶ�
float pitch_INS_speed;									//pitch�������ǽǶ�

float pitch_target_motor_speed;					//pitch����Ŀ���ٶ�
float pitch_target_INS_speed;						//pitch��������Ŀ���ٶ�
float pitch_target_INS_angle;						//pitch��������Ŀ��Ƕ�
float pitch_target_INS_speed_last;

float pitch_autoaim_target_INS_angle;		//pitch��������Ŀ��Ƕ�
float pitch_autoaim_target_INS_speed;		//pitch��������Ŀ���ٶ�

/*****������Ʋ���*****/
int16_t pitch_target_motor_current_or_torque;		//����ģʽ/����ģʽ�µ�����
uint8_t ack_status_value=2;			//��������

/*****pitch��Ŀ���ģʽ*****/
#define Pitch_RC_Mode				0								//ң��ģʽ
#define Pitch_Autoaim_Mode	1								//����ģʽ

/*****ң������ر���*****/
extern RC_ctrl_t rc_ctrl;
int16_t rc_ch1;//��ң������ֵ�������������Ŀ���Ǹ����¸����߼���ʱ��øģ�����������ƣ�ֻ��һ�������ž��ܸ�������
int16_t rc_ch1_last;
int16_t rc_mouse_y;
int16_t rc_mouse_y_last;

/*****�����λ������־*****/
uint8_t max_limit_flag=1,min_limit_flag=1;//�����λ��־����־Ϊ1�ǲ�����λ���ܣ�����һֱ���ţ������õ����ȥ

/*****����΢��*****/
int16_t KEY_C;
int16_t KEY_C_last;
int16_t KEY_B;
int16_t KEY_B_last;

/*****����*****/
extern AutoAim_Data_Rx AutoAim_Data_Receive;

/*****pitch����ģʽ�ı�־����*****/
uint8_t pitch_mode_flag;//0��ʾң��;1��ʾ����
uint8_t pitch_mode_flag_last;

float pitch_target_INS_speed_last;
extern uint8_t aim_on_off_flag;//������־λ

void Pitch_Motor_Control(void)
{
	if(	(rc_ctrl.mouse.press_r)	&&	(AutoAim_Data_Receive.If_Aimed==1)	)//����Ҽ��������鲢�������⵽Ŀ��
//	if(	((rc_ctrl.mouse.press_r)||rc_ctrl.rc.s[1]==1)	&&	(AutoAim_Data_Receive.If_Aimed==1)	)//ֻ��ң�����������ʱ������Ҽ�������������󲦸˲���������  ���������⵽Ŀ�꣨���ϼ����Ҽ��������飩
		Pitch_Motor_Control_in_Autoaim_Mode();
	else
	{
		Pitch_Motor_Control_in_RC_Mode();
	}
		
	
	pitch_mode_flag_last=pitch_mode_flag;

}


/**	[1.1]							
  * brief         ң��ģʽ�¿���pitch��
	* postscript										*/
uint8_t using_mouse_flag;
uint8_t using_rc_flag;
uint8_t using_mouse_flag_last;
uint8_t using_rc_flag_last;
uint8_t pitch_moving_flag;
uint8_t pitch_moving_flag_last;
void  Pitch_Motor_Control_in_RC_Mode()
{
			/*(0)	��ȡpitch����Ϣ������ģʽ����*/
		if(rc_ctrl.rc.s[1]==1||rc_ctrl.rc.s[1]==3)
		{
			pitch_mode_flag=Pitch_RC_Mode;//���һ��pitch��ģʽ
					
			rc_ch1=-rc_ctrl.rc.ch[1];
			rc_mouse_y=-rc_ctrl.mouse.y;
			KEY_C=((rc_ctrl.key.v & KEY_PRESSED_OFFSET_C) >> 13);
			KEY_B=((rc_ctrl.key.v & KEY_PRESSED_OFFSET_B) >> 15);//��ȡ������Ϣ
		
			pitch_INS_angle=INS_angle_deg[2];
			pitch_INS_speed =	bmi088_real_data.gyro[1];//pitch����������ٶȣ�ͷ����̧�ٶ�Ϊ����
			
		
			if(pitch_mode_flag==Pitch_RC_Mode && pitch_mode_flag_last!=Pitch_RC_Mode)		//������е�ң����ģʽ,Ŀ��Ƕ��ȶ��ڵ�ǰ�Ƕ�
				pitch_target_INS_angle=pitch_INS_angle;																		
			
			
			/*(1)	������������Ƕȷ�Χ*/
	
			if(pitch_INS_angle < PITCH_INS_MAX-INS_BUFFER  && 	pitch_INS_angle > PITCH_INS_MIN+INS_BUFFER)//�����������Ʒ�Χ��
			{ 
				
//				/*(1.1)	�����ٶȿ���*/
//				if(rc_ch1 != 0	||	rc_mouse_y!=0)//ң�������Ƶ���ٶ�	
//				{
//					pitch_target_INS_speed=PITCH_RC_SENS*	rc_ch1/660.0f+PITCH_MOUSE_SENS*rc_mouse_y;
////					pitch_target_INS_speed=0.8f*pitch_target_INS_speed_last+0.2f*pitch_target_INS_speed;
////					pitch_target_INS_speed_last=pitch_target_INS_speed;//�ӵ�ͨ�˲�
//					
//					if(rc_ch1 != 0 && rc_mouse_y==0)
//						calculate_pitch_motor_current_or_torque_with_target_INS_speed();//���ң�����ڶ��������û������ң������pid����
//					else
//						calculate_pitch_motor_current_or_torque_with_target_mouse_INS_speed();//ֻҪ����ڶ�,��������pid����
//					
//					set_motor4310_cur_tor(MOTOR_ID,pitch_target_motor_current_or_torque,0,ack_status_value);//0��ʾ��������	
//				}
//				
//				/*(1.2)	���䵽������ȷ��������Ŀ��Ƕ�*/
//				if( (rc_ch1_last!=0 && rc_ch1 == 0) || (rc_mouse_y_last!=0 && rc_mouse_y!=0))//�ж�������Ȼ�Ǵ���ģ��������ó���ִ�еļ�϶����
//					pitch_target_INS_angle=pitch_INS_angle;	
//				
//				/*(1.3)	�������ȶ��ڵ�ǰ�Ƕ�*/
//				if((rc_ch1==0) && (rc_mouse_y==0))
//				{	
//					if(KEY_C_last==0	&&	KEY_C!=0)pitch_target_INS_angle+=0.1f;
//					if(KEY_B_last==0	&&	KEY_B!=0)pitch_target_INS_angle-=0.1f;//����΢��Ŀ��ֵ
//					
//					if(rc_ch1_last!=0 && rc_ch1 == 0)
//						calculate_pitch_motor_current_or_torque_with_target_INS_angle();//������λ�û�
//					
//					if(rc_mouse_y_last!=0 && rc_mouse_y!=0)//�ж�������Ȼ�Ǵ����
//						calculate_pitch_motor_current_or_torque_with_target_mouse_INS_angle();//������λ�û�
//					
//					set_motor4310_cur_tor(MOTOR_ID,pitch_target_motor_current_or_torque,0,ack_status_value);//0��ʾ��������
//				}


			if(aim_on_off_flag==0)//���û����
				pitch_target_INS_speed=PITCH_RC_SENS*	rc_ch1/660.0f+PITCH_MOUSE_SENS*rc_mouse_y;
			else//���������
				pitch_target_INS_speed=PITCH_RC_SENS*	rc_ch1/660.0f+PITCH_MOUSE_AIM_ON_SENS*rc_mouse_y;
			 
			 pitch_target_INS_speed=0.8f*pitch_target_INS_speed_last+0.2f*pitch_target_INS_speed;
			 pitch_target_INS_speed_last=pitch_target_INS_speed;//���˸���ͨ�˲�
			 
			 if(rc_mouse_y!=0)
				 using_mouse_flag=1;
			 if(rc_ch1!=0)
				 using_rc_flag=1;
			 if(rc_mouse_y==0)
				 using_mouse_flag=0;
			 if(rc_ch1==0)
				 using_rc_flag=0;
			 
			if(rc_ch1!=0 ||rc_mouse_y!=0)	pitch_moving_flag=1;
			if(rc_ch1==0 &&	rc_mouse_y==0 && fabs(pitch_INS_speed)<0.5)	pitch_moving_flag=0;//����ң����ҡ�˺�yaw���ٶ� ��������ͬʱ�жϣ�����yaw�ٶ��жϿ��Է�ֹyaw����

			 
				if(using_mouse_flag==1)//����������
				{
					calculate_pitch_motor_current_or_torque_with_target_mouse_INS_speed();//ֻҪ����ڶ�,��������pid����
					set_motor4310_cur_tor(MOTOR_ID,pitch_target_motor_current_or_torque,0,ack_status_value);//0��ʾ��������	
				}
				
				if((pitch_moving_flag_last==1)&&(pitch_moving_flag==0))
					pitch_target_INS_angle=pitch_INS_angle;	
				
				if(using_mouse_flag==0)
				{
					/*����1����������Ŀ��λ��*/
//					if(KEY_C_last==0	&&	KEY_C!=0)pitch_target_INS_angle+=0.1f;
//					if(KEY_B_last==0	&&	KEY_B!=0)pitch_target_INS_angle-=0.1f;//����΢��Ŀ��ֵ
					
					/*����2����������Ŀ���ٶ�*/
					if(KEY_C==1)//������°���
					{
						pitch_target_INS_speed=0.01f;
						calculate_pitch_motor_current_or_torque_with_target_mouse_INS_speed();//ֻҪ����ڶ�,��������pid����
						set_motor4310_cur_tor(MOTOR_ID,pitch_target_motor_current_or_torque,0,ack_status_value);//0��ʾ��������	
					}
					if(KEY_B==1)//������°���
					{
						pitch_target_INS_speed=-0.01f;
						calculate_pitch_motor_current_or_torque_with_target_mouse_INS_speed();//ֻҪ����ڶ�,��������pid����
						set_motor4310_cur_tor(MOTOR_ID,pitch_target_motor_current_or_torque,0,ack_status_value);//0��ʾ��������	
					}					
					if(	(KEY_C_last==1	&&	KEY_C==0)	||	(KEY_B_last==1	&&	KEY_B==0)	)pitch_target_INS_angle=pitch_INS_angle;
										
					
					
					
					if(KEY_C==0 && KEY_B==0)//ң���������ȼ���ͣ������������ʱ����ֵ���
					{
						if(using_rc_flag==0)
						{
							calculate_pitch_motor_current_or_torque_with_target_mouse_INS_angle();
							set_motor4310_cur_tor(MOTOR_ID,pitch_target_motor_current_or_torque,0,ack_status_value);//0��ʾ��������	
						}
						if(using_rc_flag==1)
						{
							calculate_pitch_motor_current_or_torque_with_target_INS_speed();//���ң�����ڶ��������û������ң������pid����
							set_motor4310_cur_tor(MOTOR_ID,pitch_target_motor_current_or_torque,0,ack_status_value);//0��ʾ��������	
						}
						if(using_rc_flag_last!=0 && using_rc_flag==0)
							pitch_target_INS_angle=pitch_INS_angle;	
					}

				}

			
			 using_mouse_flag_last=using_mouse_flag;
			 using_rc_flag_last=using_rc_flag;
				pitch_moving_flag_last=pitch_moving_flag;
			
			
				
				/*(1.4)	���������Ƕȷ�Χ�ڣ�����������λ����*/		
				max_limit_flag=min_limit_flag=1;
			}
			
			
						/*(2)	��������λ�Ƕȸ���*/
			
			if(pitch_INS_angle >= PITCH_INS_MAX-INS_BUFFER)//������������λ������
			{
				/*(2.1)	�����λ���ܿ����������λ��Ȼ��ص���λ����*/
				if(max_limit_flag==1)
				{
					pitch_target_INS_angle=PITCH_INS_MAX;
					calculate_pitch_motor_current_or_torque_with_target_INS_angle();//������λ�û�
					set_motor4310_cur_tor(MOTOR_ID,pitch_target_motor_current_or_torque,0,ack_status_value);//0��ʾ��������

					max_limit_flag=0;
				}
				
				/*(2.2)	���������Ƶ������λ�����ߣ��ٿ�����λ���ܣ�������λ*/		
				if(rc_ch1<0 || rc_mouse_y<0) 
					max_limit_flag=1; 		
				
				/*(2.3)	������Ƶ���뿪��λ�����������ٶȿ���������*/		
//				if(rc_ch1>=0 || rc_mouse_y>=0)
				else
				{
					pitch_target_INS_speed=PITCH_RC_SENS*	rc_ch1/660.0f+PITCH_MOUSE_SENS*rc_mouse_y;
					calculate_pitch_motor_current_or_torque_with_target_INS_speed();//�������ٶȻ�
					set_motor4310_cur_tor(MOTOR_ID,pitch_target_motor_current_or_torque,0,ack_status_value);//0��ʾ��������	
				}
			}
			
			
			/*(3)	�����С��λ�Ƕȸ���*/		
			
			if(pitch_INS_angle<=PITCH_INS_MIN+INS_BUFFER)//������������λ������
			{
				/*(3.1)	�����λ���ܿ����������λ��Ȼ��ص���λ����*/
				if(min_limit_flag==1)
				{
					pitch_target_INS_angle=PITCH_INS_MIN;
					calculate_pitch_motor_current_or_torque_with_target_INS_angle();//������λ�û�
					set_motor4310_cur_tor(MOTOR_ID,pitch_target_motor_current_or_torque,0,ack_status_value);//0��ʾ��������
					
					min_limit_flag=0;
				}
				
				/*(3.2)	���������Ƶ������λ�����ߣ��ٿ�����λ���ܣ�������λ*/	
				if(rc_ch1>0 || rc_mouse_y>0)	
					min_limit_flag=1;	
				
				/*(3.3)	������Ƶ���뿪��λ�����������ٶȿ���������*/		
//				if(rc_ch1<=0 || rc_mouse_y<=0)
				else
				{
					pitch_target_INS_speed=PITCH_RC_SENS*	rc_ch1/660.0f+PITCH_MOUSE_SENS*rc_mouse_y;
					calculate_pitch_motor_current_or_torque_with_target_INS_speed();//�������ٶȻ�
					set_motor4310_cur_tor(MOTOR_ID,pitch_target_motor_current_or_torque,0,ack_status_value);//0��ʾ��������			
				}				
			}
			
			
			
			rc_ch1_last=rc_ch1;
			rc_mouse_y_last=rc_mouse_y;
			KEY_C_last=KEY_C;
			KEY_B_last=KEY_B;
		}
		else
		{
			set_motor4310_cur_tor(MOTOR_ID,0,0,ack_status_value);//0��ʾ��������
		}
		
}

/**	[1.2]							
  * brief         ң��ģʽ�¿���pitch��
	* postscript										*/
static float pitch_last;
void  Pitch_Motor_Control_in_Autoaim_Mode()
{
		pitch_mode_flag=1;		//���һ��pitch��ģʽ
	
		pitch_INS_angle=INS_angle_deg[2];//pitch�Ƕȶ�ȡ��̬������ĽǶ�����
		pitch_INS_speed =	bmi088_real_data.gyro[1];//pitch�ٶȶ�ȡ������ֱ�Ӳ�����Ľ��ٶ����ݣ������ǲ����������ٶ�
	
		pitch_autoaim_target_INS_angle=-AutoAim_Data_Receive.Pitch;//�Ӿ���������pitch��ͷ���ˮƽ��ļнǣ�C��ŷ��ˣ��üӸ����ţ���ʱû��ǰ��
		pitch_autoaim_target_INS_angle=0.8f*pitch_last+pitch_autoaim_target_INS_angle*0.2f;
		pitch_last=pitch_autoaim_target_INS_angle;//�ӵ�ͨ�˲�
	
		if(pitch_autoaim_target_INS_angle>PITCH_INS_MAX)pitch_autoaim_target_INS_angle=PITCH_INS_MAX;
		if(pitch_autoaim_target_INS_angle<PITCH_INS_MIN)pitch_autoaim_target_INS_angle=PITCH_INS_MIN;//������λ
		
		calculate_pitch_motor_current_or_torque_with_autoaim_target_INS_angle();
		set_motor4310_cur_tor(MOTOR_ID,pitch_target_motor_current_or_torque,0,ack_status_value);//0��ʾ��������	

}

/**[2.1]						
  * brief						����λ�û�							
	* postscript						*/
void calculate_pitch_motor_current_or_torque_with_autoaim_target_INS_angle()
{
		pitch_autoaim_target_INS_speed=-PID_calc(&pitch_motor_autoaim_INS_angle_pid,	pitch_INS_angle,pitch_autoaim_target_INS_angle);
		pitch_target_motor_current_or_torque=PID_calc(&pitch_motor_autoaim_INS_speed_pid,	pitch_INS_speed,pitch_autoaim_target_INS_speed);
}

/**[2.2]					
  * brief					ң���������ٶȻ�											
	* postscript							*/
void calculate_pitch_motor_current_or_torque_with_target_INS_speed()
{
		pitch_target_motor_current_or_torque=PID_calc(&pitch_motor_INS_speed_pid,	pitch_INS_speed,pitch_target_INS_speed);
}

/**[2.3]					
  * brief					ң��������λ�û�											
	* postscript							*/
void calculate_pitch_motor_current_or_torque_with_target_INS_angle()
{
		pitch_target_INS_speed=-PID_calc(&pitch_motor_INS_angle_pid,	pitch_INS_angle,pitch_target_INS_angle);
		pitch_target_motor_current_or_torque=PID_calc(&pitch_motor_INS_speed_pid,	pitch_INS_speed,pitch_target_INS_speed);
}

/**[2.4]					
  * brief					����������ٶȻ�											
	* postscript							*/
void calculate_pitch_motor_current_or_torque_with_target_mouse_INS_speed()
{
		pitch_target_motor_current_or_torque=PID_calc(&pitch_motor_mouse_INS_speed_pid,	pitch_INS_speed,pitch_target_INS_speed);
}
/**[2.5]					
  * brief					���������λ�û�											
	* postscript							*/
void calculate_pitch_motor_current_or_torque_with_target_mouse_INS_angle()
{
		pitch_target_INS_speed=-PID_calc(&pitch_motor_mouse_INS_angle_pid,	pitch_INS_angle,pitch_target_INS_angle);
		pitch_target_motor_current_or_torque=PID_calc(&pitch_motor_mouse_INS_speed_pid,	pitch_INS_speed,pitch_target_INS_speed);
}
/**	[3]							
  * brief         	pitch��pid��ʼ��
	* postscript			��ʼ�������������Ϣ�Ϳ�����Ϣ							*/
void pitch_motor_init(void)
{
	pitch_motor_INS_speed_PID[0]	=	pitch_motor_INS_speed_pid_Kp;
	pitch_motor_INS_speed_PID[1]	=	pitch_motor_INS_speed_pid_Ki;
	pitch_motor_INS_speed_PID[2]	=	pitch_motor_INS_speed_pid_Kd;
	PID_init(	&pitch_motor_INS_speed_pid,	PID_POSITION,	pitch_motor_INS_speed_PID,	pitch_motor_INS_speed_pid_max_out,	pitch_motor_INS_speed_pid_max_iout);
	
	pitch_motor_INS_angle_PID[0]	=	pitch_motor_INS_angle_pid_Kp;
	pitch_motor_INS_angle_PID[1]	=	pitch_motor_INS_angle_pid_Ki;
	pitch_motor_INS_angle_PID[2]	=	pitch_motor_INS_angle_pid_Kd;
	PID_init(	&pitch_motor_INS_angle_pid,	PID_POSITION,	pitch_motor_INS_angle_PID,	pitch_motor_INS_angle_pid_max_out,	pitch_motor_INS_angle_pid_max_iout);
	
	pitch_motor_autoaim_INS_speed_PID[0]	=	pitch_motor_autoaim_INS_speed_pid_Kp;
	pitch_motor_autoaim_INS_speed_PID[1]	=	pitch_motor_autoaim_INS_speed_pid_Ki;
	pitch_motor_autoaim_INS_speed_PID[2]	=	pitch_motor_autoaim_INS_speed_pid_Kd;
	PID_init(	&pitch_motor_autoaim_INS_speed_pid,	PID_POSITION,	pitch_motor_autoaim_INS_speed_PID,	pitch_motor_autoaim_INS_speed_pid_max_out,	pitch_motor_autoaim_INS_speed_pid_max_iout);

	pitch_motor_autoaim_INS_angle_PID[0]	=	pitch_motor_autoaim_INS_angle_pid_Kp;
	pitch_motor_autoaim_INS_angle_PID[1]	=	pitch_motor_autoaim_INS_angle_pid_Ki;
	pitch_motor_autoaim_INS_angle_PID[2]	=	pitch_motor_autoaim_INS_angle_pid_Kd;
	PID_init(	&pitch_motor_autoaim_INS_angle_pid,	PID_POSITION,	pitch_motor_autoaim_INS_angle_PID,	pitch_motor_autoaim_INS_angle_pid_max_out,	pitch_motor_autoaim_INS_angle_pid_max_iout);

	pitch_motor_mouse_INS_speed_PID[0]	=	pitch_motor_mouse_INS_speed_pid_Kp;
	pitch_motor_mouse_INS_speed_PID[1]	=	pitch_motor_mouse_INS_speed_pid_Ki;
	pitch_motor_mouse_INS_speed_PID[2]	=	pitch_motor_mouse_INS_speed_pid_Kd;
	PID_init(	&pitch_motor_mouse_INS_speed_pid,	PID_POSITION,	pitch_motor_mouse_INS_speed_PID,	pitch_motor_mouse_INS_speed_pid_max_out,	pitch_motor_mouse_INS_speed_pid_max_iout);
	
	pitch_motor_mouse_INS_angle_PID[0]	=	pitch_motor_mouse_INS_angle_pid_Kp;
	pitch_motor_mouse_INS_angle_PID[1]	=	pitch_motor_mouse_INS_angle_pid_Ki;
	pitch_motor_mouse_INS_angle_PID[2]	=	pitch_motor_mouse_INS_angle_pid_Kd;
	PID_init(	&pitch_motor_mouse_INS_angle_pid,	PID_POSITION,	pitch_motor_mouse_INS_angle_PID,	pitch_motor_mouse_INS_angle_pid_max_out,	pitch_motor_mouse_INS_angle_pid_max_iout);
}



/*********************************************************���Գ���*********************************************************/
//	/*���Գ���1������ģʽ��������*/
//	if(rc_ctrl.rc.s[0]==3)
//	set_motor4310_cur_tor(MOTOR_ID,current_or_torque_expected,0,ack_status_value);//0��ʾ��������
//	if(rc_ctrl.rc.s[0]==2)
//	set_motor4310_cur_tor(MOTOR_ID,0,0,ack_status_value);//0��ʾ��������
	
//		/*���Գ���2������ٶȻ�����*/
//	if(rc_ctrl.rc.s[0]==1)
//	{
//		pitch_INS_speed =	bmi088_real_data.gyro[1];//pitch����������ٶȣ�ͷ����̧�ٶ�Ϊ����
//		pitch_target_INS_speed=0.5f;
//		calculate_pitch_motor_current_or_torque_with_target_INS_speed();
//		set_motor4310_cur_tor(MOTOR_ID,pitch_target_motor_current_or_torque,0,ack_status_value);//0��ʾ��������
//	}
//	if(rc_ctrl.rc.s[0]==3)
//	{
//		pitch_INS_speed =	bmi088_real_data.gyro[1];//pitch����������ٶȣ�ͷ����̧�ٶ�Ϊ����
//		pitch_target_INS_speed=0.0f;
//		calculate_pitch_motor_current_or_torque_with_target_INS_speed();
//		set_motor4310_cur_tor(MOTOR_ID,pitch_target_motor_current_or_torque,0,ack_status_value);//0��ʾ��������
//	}
//	if(rc_ctrl.rc.s[0]==2)
//	{
//		pitch_INS_speed =	bmi088_real_data.gyro[1];//pitch����������ٶȣ�ͷ����̧�ٶ�Ϊ����
//		pitch_target_INS_speed=-0.5f;
//		calculate_pitch_motor_current_or_torque_with_target_INS_speed();
//		set_motor4310_cur_tor(MOTOR_ID,pitch_target_motor_current_or_torque,0,ack_status_value);//0��ʾ��������
//		
////			set_motor4310_cur_tor(MOTOR_ID,0,0,ack_status_value);//0��ʾ��������
//	}

	
//		/*���Գ���3�����λ�û�����*/
//		if(rc_ctrl.rc.s[0]==3)
//		{
//			pitch_INS_angle=INS_angle_deg[2];
//			pitch_target_INS_angle=0;
//			pitch_INS_speed =	bmi088_real_data.gyro[1];
//			calculate_pitch_motor_current_or_torque_with_target_INS_angle();//��仰֮ǰ�ǲ�������һ����	pitch_INS_speed =	bmi088_real_data.gyro[1];��??
//			set_motor4310_cur_tor(MOTOR_ID,pitch_target_motor_current_or_torque,0,ack_status_value);//0��ʾ��������
//		}
//		if(rc_ctrl.rc.s[0]==2)
//		set_motor4310_cur_tor(MOTOR_ID,0,0,ack_status_value);//0��ʾ��������


//			/*���Գ���4������λ�û���Ծ����*/
//			pitch_INS_angle=INS_angle_deg[2];
//			pitch_INS_speed =	bmi088_real_data.gyro[1];//pitch����������ٶȣ�ͷ����̧�ٶ�Ϊ����
//			if(rc_ctrl.rc.s[0]==1)
//				pitch_autoaim_target_INS_angle=-30.0f;
//			if(rc_ctrl.rc.s[0]==3)
//				pitch_autoaim_target_INS_angle=5.0f;
//			if(rc_ctrl.rc.s[0]==2)
//				pitch_autoaim_target_INS_angle=0.0f;
//			calculate_pitch_motor_current_or_torque_with_autoaim_target_INS_angle();
//			set_motor4310_cur_tor(MOTOR_ID,pitch_target_motor_current_or_torque,0,ack_status_value);//0��ʾ��������	


//				/*���Գ���5��������*/
//				pitch_INS_angle=INS_angle_deg[2];
//				pitch_INS_speed =	bmi088_real_data.gyro[1];//pitch����������ٶȣ�ͷ����̧�ٶ�Ϊ����
//				if(AutoAim_Data_Receive.If_Aimed==1)
//				{
//					pitch_autoaim_target_INS_angle=-AutoAim_Data_Receive.Pitch;
//				/*������λ*/
//				if(pitch_autoaim_target_INS_angle>PITCH_INS_MAX)pitch_autoaim_target_INS_angle=PITCH_INS_MAX;
//				if(pitch_autoaim_target_INS_angle<PITCH_INS_MIN)pitch_autoaim_target_INS_angle=PITCH_INS_MIN;
//				
//				calculate_pitch_motor_current_or_torque_with_autoaim_target_INS_angle();
//				set_motor4310_cur_tor(MOTOR_ID,pitch_target_motor_current_or_torque,0,ack_status_value);//0��ʾ��������	
//				}
//				else
//					set_motor4310_cur_tor(MOTOR_ID,0,0,ack_status_value);//0��ʾ��������	
				
//				/*���Գ���6��pitch����-23��λ�ã����yaw�������pid*/
//				pitch_INS_angle=INS_angle_deg[2];
//				pitch_INS_speed =	bmi088_real_data.gyro[1];
//				pitch_autoaim_target_INS_angle=-23.0f;
//				calculate_pitch_motor_current_or_torque_with_autoaim_target_INS_angle();
//				set_motor4310_cur_tor(MOTOR_ID,pitch_target_motor_current_or_torque,0,ack_status_value);//0��ʾ��������	




/*********************************************************˵�����ϵ�Э��Ϳ��ƺ���*********************************************************/
union RV_TypeConvert
{
	float to_float;
	int to_int;
	unsigned int to_uint;
	uint8_t buf[4];
}rv_type_convert;

union RV_TypeConvert2
{
	int16_t to_int16;
	uint16_t to_uint16;
	uint8_t buf[2];
}rv_type_convert2;


    
    
float uint_to_float(int x_int, float x_min, float x_max, int bits){
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
    }


/*****************************************[1]	�������ָ��*****************************************************************/
		
/**	[1.1]				
  * brief       ���õ��ͨ��ģʽ  	
  * param[in]		motor_id:	���ID
  * param[in]		cmd:	0x01���Զ�����ģʽ��	0x02���ʴ�ͨ��ģʽ      
  * postscript	�������ָ��ı�ʶ������0x7FF�����óɹ�����е����ı��ķ�������ʽ���ֲᡰ2.1 ���ͨ��ģʽ����ָ�			*/
void set_motor4310_comm_mode(uint16_t motor_id,uint8_t cmd)
{
		if(cmd==0) return;//���cmd��0������ʧ�ܣ�����
	
	
		CAN_TxHeaderTypeDef  motor_comm_mode_setting_tx_message;//comm:communication
		uint8_t              motor_comm_mode_setting_can_send_data[4];
		uint32_t  send_mail_box;
		motor_comm_mode_setting_tx_message.StdId=0x7FF;//�ٲó�����׼֡��11λID��ID��identifier��ʶ��   Ŀ���豸��ID��  000-7FF��Χ��2��11�η�
		motor_comm_mode_setting_tx_message.RTR=CAN_RTR_DATA;//�ٲó���RTR��Remote Transmission Request��Զ�̷�������RTRλ��������֡��Զ��֡
		motor_comm_mode_setting_tx_message.IDE=CAN_ID_STD;//���Ƴ���IDE:Identifier Extension����ʶ����չ��0��ʾ������չ�����Ǳ�׼��ʶ����1��ʾ��չ��ʶ��
		motor_comm_mode_setting_tx_message.DLC=0x04;//���Ƴ���     DLC��Data Length Code���ݳ��ȴ���
		
		motor_comm_mode_setting_can_send_data[0]=motor_id>>8;
		motor_comm_mode_setting_can_send_data[1]=motor_id&0xff;
		motor_comm_mode_setting_can_send_data[2]=0x00;
		motor_comm_mode_setting_can_send_data[3]=cmd;
	
		HAL_CAN_AddTxMessage(&MOTOR4310_CAN, &motor_comm_mode_setting_tx_message, motor_comm_mode_setting_can_send_data, &send_mail_box);
}

/**	[1.2]			
  * brief        	���õ��IDΪ0x01  
  * postscript		�������ָ��ı�ʶ������0x7FF�����óɹ�����е����ı��ķ���	����ʽ���ֲᡰ2.3 ��� CAN ͨ�� ID ����ָ�		*/
void reset_motor4310_id(void)
{
	CAN_TxHeaderTypeDef  motor_id_reset_tx_message;//com:communication
	uint8_t              motor_id_reset_can_send_data[6];
	uint32_t send_mail_box;
	motor_id_reset_tx_message.StdId=0x7ff;
	motor_id_reset_tx_message.RTR=CAN_RTR_DATA;
	motor_id_reset_tx_message.IDE=CAN_ID_STD;
	motor_id_reset_tx_message.DLC=0x06;
	
	motor_id_reset_can_send_data[0]=0x7F;
	motor_id_reset_can_send_data[1]=0x7F;
	motor_id_reset_can_send_data[2]=0x00;
	motor_id_reset_can_send_data[3]=0x05;
	motor_id_reset_can_send_data[4]=0x7F;
	motor_id_reset_can_send_data[5]=0x7F;
	
	HAL_CAN_AddTxMessage(&MOTOR4310_CAN,&motor_id_reset_tx_message,motor_id_reset_can_send_data,&send_mail_box);
}

/**	[1.3]			
	*	brief					���õ��ͨ��ID
  * param[in]			motor_id:���ID��
	*	param[in]			motor_id_new:�����ID	      
  * postscript	  �������ָ��ı�ʶ������0x7FF�����óɹ��е����ı��ķ�����ʧ������Ӧ�𣬸�ʽ���ֲᡰ2.4 ��� CAN ͨ�� ID ����ָ�			*/
void set_motor4310_id(uint16_t motor_id,uint16_t motor_id_new)
{
	CAN_TxHeaderTypeDef motor_id_setting_tx_message;
	uint8_t motor_id_setting_can_send_data[6];
	uint32_t send_mail_box;
	motor_id_setting_tx_message.StdId=0x7FF;
	motor_id_setting_tx_message.RTR=CAN_RTR_DATA;
	motor_id_setting_tx_message.IDE=CAN_ID_STD;
	motor_id_setting_tx_message.DLC=0x06;
	
	motor_id_setting_can_send_data[0]=motor_id>>8;
	motor_id_setting_can_send_data[1]=motor_id&0xff;
	motor_id_setting_can_send_data[2]=0x00;
	motor_id_setting_can_send_data[3]=0x04;
	motor_id_setting_can_send_data[4]=motor_id_new>>8;
	motor_id_setting_can_send_data[5]=motor_id_new&0xff;
	
	HAL_CAN_AddTxMessage(&MOTOR4310_CAN,&motor_id_setting_tx_message,motor_id_setting_can_send_data,&send_mail_box);
	
}

/*****************************************[2] �����ѯָ��*****************************************************************/

/**	[2.1]			
	* brief					��ѯ���ͨ��ģʽ
  * param[in]			motor_id:���ID��	      
  * postscript		�����ѯָ��ı�ʶ������0x7FF����ѯ�ɹ�����е����ı��ķ�������ʽ���ֲᡰ3.1 ��ѯ���ͨ��ģʽ��			*/
void read_motor4310_comm_mode(uint16_t motor_id)
{
	CAN_TxHeaderTypeDef motor_comm_mode_tx_message;
	uint8_t motor_comm_mode_can_send_data[4];
	uint32_t send_mail_box;
	motor_comm_mode_tx_message.StdId=0x7ff;
	motor_comm_mode_tx_message.RTR=CAN_RTR_DATA;
	motor_comm_mode_tx_message.IDE=CAN_ID_STD;
	motor_comm_mode_tx_message.DLC=0x04;
	
	motor_comm_mode_can_send_data[0]=motor_id>>8;
	motor_comm_mode_can_send_data[1]=motor_id&0xff;
	motor_comm_mode_can_send_data[2]=0x00;
	motor_comm_mode_can_send_data[3]=0x81;
	
	HAL_CAN_AddTxMessage(&MOTOR4310_CAN,&motor_comm_mode_tx_message,motor_comm_mode_can_send_data,&send_mail_box);
	
}

/**	[2.2]			
  * brief			    ��ѯ���ͨ��ID
  * postscript		�����ѯָ��ı�ʶ������0x7FF����ѯ�ɹ�����е����ı��ķ�������ʽ���ֲᡰ3.2 ��ѯCAN ͨ�� ID��			*/
void read_motor4310_id(void)
{
		CAN_TxHeaderTypeDef motor_id_reading_tx_message;
		uint8_t motor_id_reading_can_send_data[4];
		uint32_t send_mail_box;
		motor_id_reading_tx_message.StdId=0x7ff;
		motor_id_reading_tx_message.RTR=CAN_RTR_DATA;
		motor_id_reading_tx_message.IDE=CAN_ID_STD;
		motor_id_reading_tx_message.DLC=0x04;
	
		motor_id_reading_can_send_data[0]=0xff;
		motor_id_reading_can_send_data[1]=0xff;
		motor_id_reading_can_send_data[2]=0x00;
		motor_id_reading_can_send_data[3]=0x82;
		
		HAL_CAN_AddTxMessage(&MOTOR4310_CAN,&motor_id_reading_tx_message,motor_id_reading_can_send_data,&send_mail_box);
}


/*****************************************[3] �������ָ��*****************************************************************/

/**********************************[3.1] �������ָ��(�Զ�����ͨ��ģʽ)****************************************/
/**	[3.1]					�Զ�����ͨ��ģʽ
  * brief         �������ָ��
  * param[in]			motor4310_data_t motor4310_data[8]������˸�������ݵĽṹ������
  * param[in]			uint8_t motor_quantity��  �������1~8    
  * postscript		��ͨ��Э���һ֡���ݿ������ 4 ����������ǽ��ܿ��Ƶ���ĵ���	��������պ���Զ����ر��ģ���ʽ���ֲᡰ4.1 �Զ�����ͨ��ģʽ����ָ�		*/
void set_motors4310_current(motor4310_data_t motor4310_data[8],uint8_t motor_quantity)
{
	uint16_t i;

	CAN_TxHeaderTypeDef motor4310_current_setting_tx_message;
	uint8_t motor4310_current_setting_can_send_data[8];
	uint32_t send_mail_box;
	
	/**(0)	����Ҫ���͵�����֡�Ĳ��ֲ���**/
	motor4310_current_setting_tx_message.RTR = CAN_RTR_DATA;
	motor4310_current_setting_tx_message.IDE = CAN_ID_STD;
	motor4310_current_setting_tx_message.DLC = 8;
	
	
	
	/**(1)	����ǰ�ĸ����can����ָ��**/
	
		/*(1.1)	ȥ���������Ϊ������*/
	if(motor_quantity<1)return;//������������0��û��Ҫ����ȥ�ˣ����ذ�
		/*(1.2)	��������֡�ı�ʶ��*/
	motor4310_current_setting_tx_message.StdId = 0x1FF;
		/*(1.3)	��������֡�ĵ�������*/
	for(i=0;i<4;i++)
	{
		motor4310_current_setting_can_send_data[2*i]  = motor4310_data[i].current_desired_int>>8;//�ĸ������������ֵ��8λ
		motor4310_current_setting_can_send_data[2*i+1]= motor4310_data[i].current_desired_int&0xff;//�ĸ������������ֵ��8λ
	}
	i=0;//����i�����㣬���������
		/*(1.4)	����can����֡*/
	while((HAL_CAN_AddTxMessage(&MOTOR4310_CAN,&motor4310_current_setting_tx_message,motor4310_current_setting_can_send_data,&send_mail_box)==HAL_ERROR)&&(i<0XFFF))
	{
		i++;	//�ȴ����ͽ���
		if(i>=0XFFF)	break;
	}
	
	
	
	/**(2)	���ͺ��ĸ����can����ָ��**/
	
		/*(2.1)	ȥ���������Ϊ������*/
	if(motor_quantity<5)	return;//����������С��5��ǰ�淢�ĸ�����Ѿ����ˣ�Ҳû��Ҫ�������ĸ��ˣ����ذ�
		/*(2.2)	��������֡�ı�ʶ��*/
	motor4310_current_setting_tx_message.StdId = 0x2FF;
		/*(2.3)	��������֡�ĵ�������*/
	for(i=0;i<4;i++)
	{
		motor4310_current_setting_can_send_data[2*i]  = motor4310_data[i+4].current_desired_int>>8;//�����ĸ������������ֵ��8λ
		motor4310_current_setting_can_send_data[2*i+1]= motor4310_data[i+4].current_desired_int&0xff;//�����ĸ������������ֵ��8λ
	}
	i=0;
		/*(2.4)	����can����֡*/
	while((HAL_CAN_AddTxMessage(&MOTOR4310_CAN,&motor4310_current_setting_tx_message,motor4310_current_setting_can_send_data,&send_mail_box)==HAL_ERROR)&&(i<0XFFF))
	{
		i++;	//�ȴ����ͽ���
		if(i>=0XFFF)	break;
	}
	
}

/**********************************[3.2] �������ָ��(�ʴ�ͨ��ģʽ)****************************************/
/********************[3.2.1] �������ָ��(�ʴ�ͨ��ģʽ)(��ͬ����ģʽ)***********************/
/**	[3.2.1.1]			�ʴ�ͨ��ģʽ
  * brief         �������ָ��(�ʴ�ͨ��ģʽ)(��λ���ģʽ)
  * param[in]			motor_id:1~0x7FE
  * param[in]			kp:0~4095 	��Ӧ 0.0f~500.0f��
  * param[in]			kd:0~511  	��Ӧ 0.0f~5.0f
  * param[in]			pos:0~65536 ��Ӧ-12.5rad~12
  * param[in]			spd:0~4095 	��Ӧ-18.0rad/s~18.0rad/s
  * param[in]			tor:0~4095 	��Ӧ-30.0Nm~30.0Nm
  * postscript		�ʴ�ͨ��ģʽ�µĵ������ָ���ܷ���1~3���ֱ������ͣ�������λ���ģʽֻ���ر�������1���޷�ѡ�񷵻��������ͣ���ʽ���ֲᡰ5 �ʴ�ģʽ�������ġ�		*/
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 50.0f
#define POS_MIN -12.5f
#define POS_MAX 12.5f
#define SPD_MIN -18.0f
#define SPD_MAX 18.0f
#define T_MIN -30.0f
#define T_MAX 30.0f
#define I_MIN -30.0f
#define I_MAX 30.0f
/* �ڸ�����Χ��λ��������£���������ת��Ϊ�޷��������� */
int float_to_uint(float x, float x_min, float x_max, int bits)
{    
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);  
}
void send_motor4310_ctrl_cmd(uint16_t motor_id,float kp,float kd,float pos,float spd,float tor)
{
	int kp_int;
	int kd_int;
	int pos_int;            
	int spd_int;
	int tor_int;
	
	/*(1)	�����޷�*/
	if(kp>KP_MAX) 	kp=KP_MAX;	if(kp<KP_MIN) 	kp=KP_MIN;
	if(kd>KD_MAX) 	kd=KD_MAX;	if(kd<KD_MIN) 	kd=KD_MIN;	
	if(pos>POS_MAX)	pos=POS_MAX;if(pos<POS_MIN) pos=POS_MIN;
	if(spd>SPD_MAX)	spd=SPD_MAX;if(spd<SPD_MIN) spd=SPD_MIN;
	if(tor>T_MAX)		tor=T_MAX;	if(tor<T_MIN) 	tor=T_MIN;
		
	/*(2)	��float������ת��uint�ͣ���Ϊcanͨ��ֻ�ܷ�uint��*/
  kp_int	= float_to_uint(	kp,		KP_MIN,		KP_MAX,		12	);//float������ת����unit12������
  kd_int	= float_to_uint(	kd,		KD_MIN,		KD_MAX, 	9		);
	pos_int = float_to_uint(	pos,	POS_MIN,	POS_MAX, 	16	);            
  spd_int = float_to_uint(	spd,	SPD_MIN, 	SPD_MAX, 	12	);
  tor_int = float_to_uint(	tor,	T_MIN, 		T_MAX, 		12	);	
	
	
	CAN_TxHeaderTypeDef motor_ctrl_cmd_tx_message;
	uint8_t motor_ctrl_cmd_can_send_data[8];
	uint32_t send_mail_box;
	
	/*(3)	��������֡���Ͳ���*/
	motor_ctrl_cmd_tx_message.StdId = motor_id;//��ʶ��Ϊ���ID
	motor_ctrl_cmd_tx_message.IDE = CAN_ID_STD;
	motor_ctrl_cmd_tx_message.RTR = CAN_RTR_DATA;
	motor_ctrl_cmd_tx_message.DLC = 8;
	
	/*(4)	��������֡��������*/																																				
  motor_ctrl_cmd_can_send_data[0]=0x00|(kp_int>>7);											
	motor_ctrl_cmd_can_send_data[1]=((kp_int&0x7F)<<1)|((kd_int&0x100)>>8);
	motor_ctrl_cmd_can_send_data[2]=kd_int&0xFF;
	motor_ctrl_cmd_can_send_data[3]=pos_int>>8;
	motor_ctrl_cmd_can_send_data[4]=pos_int&0xFF;
	motor_ctrl_cmd_can_send_data[5]=spd_int>>4;
	motor_ctrl_cmd_can_send_data[6]=(spd_int&0x0F)<<4|(tor_int>>8);
	motor_ctrl_cmd_can_send_data[7]=tor_int&0xff;
	/*(4)	��������֡*/
  HAL_CAN_AddTxMessage(&MOTOR4310_CAN,&motor_ctrl_cmd_tx_message,motor_ctrl_cmd_can_send_data,&send_mail_box);
	
}

/**	[3.2.1.2]			�ʴ�ͨ��ģʽ
  * brief         �������ָ��(�ʴ�ͨ��ģʽ)(�ŷ�λ�ÿ���ģʽ)
  * param[in]			motor_id:1~0x7FE
  * param[in]			pos:���������λ�ã���ֵ��Ϊʵ�ʽǶȣ���λΪ���ȣ�����120.5 ��ʾ120.5��
  * param[in]			spd:����������ٶȣ�0~32767 ��Ӧ 0~3276.7rpm������Ϊ 10
  * param[in]			cur:���������ֵ��0~4095 ��Ӧ 0~409.5A������Ϊ 10
	* param[in]			ack_status�����ķ���״̬��0��������  1�����ر�������1  2�����ر�������2  3�����ر�������3
  * postscript		֮ǰ��ע��д�ģ�spd:0~18000	��cur:0~3000	���ʴ�ͨ��ģʽ�µĵ������ָ���ܷ���1~3���ֱ������ͣ���ʽ���ֲᡰ5 �ʴ�ģʽ�������ġ�	*/
void set_motor4310_position(uint16_t motor_id,float pos,uint16_t spd,uint16_t cur,uint8_t ack_status)
{
	CAN_TxHeaderTypeDef motor_pos_setting_tx_message;
	uint8_t motor_pos_setting_can_send_data[8];
	uint32_t send_mail_box;
	motor_pos_setting_tx_message.StdId = motor_id;
	motor_pos_setting_tx_message.RTR = CAN_RTR_DATA;
	motor_pos_setting_tx_message.IDE = CAN_ID_STD;
	motor_pos_setting_tx_message.DLC = 8;
	
	/*���ķ���״ֻ̬��0��3*/
	if(ack_status>3) 	return;
	
	rv_type_convert.to_float=pos;
  motor_pos_setting_can_send_data[0]=0x20|(rv_type_convert.buf[3]>>3);
	motor_pos_setting_can_send_data[1]=(rv_type_convert.buf[3]<<5)|(rv_type_convert.buf[2]>>3);
	motor_pos_setting_can_send_data[2]=(rv_type_convert.buf[2]<<5)|(rv_type_convert.buf[1]>>3);
	motor_pos_setting_can_send_data[3]=(rv_type_convert.buf[1]<<5)|(rv_type_convert.buf[0]>>3);
	motor_pos_setting_can_send_data[4]=(rv_type_convert.buf[0]<<5)|(spd>>10);
	motor_pos_setting_can_send_data[5]=(spd&0x3FC)>>2;
	motor_pos_setting_can_send_data[6]=(spd&0x03)<<6|(cur>>6);
	motor_pos_setting_can_send_data[7]=(cur&0x3F)<<2|ack_status;
	
	HAL_CAN_AddTxMessage(&MOTOR4310_CAN,&motor_pos_setting_tx_message,motor_pos_setting_can_send_data,&send_mail_box); 
}

/**	[3.2.1.3]			�ʴ�ͨ��ģʽ
  * brief         �������ָ��(�ʴ�ͨ��ģʽ)(�ŷ��ٶȿ���ģʽ)
  * param[in]			motor_id:1~0x7FE
  * param[in]			spd:����������ٶȣ�CAN ��ֵ��ʵ��ת��һһ��Ӧ
  * param[in]			cur:���������ֵ��0~65536 ��Ӧ 0~6553.6A������Ϊ10
	* param[in]			ack_status�����ķ���״̬��0��������  1�����ر�������1  2�����ر�������2  3�����ر�������3
  * postscript		���ŷ�λ�ÿ��Ƶ���ֵ���ͻ���һ��������֮ǰ��ע��д�ģ�spd:-18000~18000	��cur:0~3000���ʴ�ͨ��ģʽ�µĵ������ָ���ܷ���1~3���ֱ������ͣ���ʽ���ֲᡰ5 �ʴ�ģʽ�������ġ�		*/
void set_motor4310_speed(uint16_t motor_id,float spd,uint16_t cur,uint8_t ack_status)
{
	CAN_TxHeaderTypeDef motor_spd_setting_tx_message;
	uint8_t motor_spd_setting_can_send_data[7];
	uint32_t send_mail_box;
	motor_spd_setting_tx_message.IDE = CAN_ID_STD;//CAN_ID_EXT
	motor_spd_setting_tx_message.StdId = motor_id;
	motor_spd_setting_tx_message.RTR = CAN_RTR_DATA;
	motor_spd_setting_tx_message.DLC = 7;
	
	rv_type_convert.to_float=spd;
  motor_spd_setting_can_send_data[0]=0x40|ack_status;
	motor_spd_setting_can_send_data[1]=rv_type_convert.buf[3];
	motor_spd_setting_can_send_data[2]=rv_type_convert.buf[2];
	motor_spd_setting_can_send_data[3]=rv_type_convert.buf[1];
	motor_spd_setting_can_send_data[4]=rv_type_convert.buf[0];
	motor_spd_setting_can_send_data[5]=cur>>8;
	motor_spd_setting_can_send_data[6]=cur&0xff;
	
 HAL_CAN_AddTxMessage(&MOTOR4310_CAN,&motor_spd_setting_tx_message,motor_spd_setting_can_send_data,&send_mail_box);
}

/**	[3.2.1.4]			�ʴ�ͨ��ģʽ
  * brief         �������ָ��(�ʴ�ͨ��ģʽ)(���������ؿ���ģʽ��ɲ������ָ��)
  * param[in]			motor_id:1~0x7FE
  * param[in]			cur_tor:     ����������-32768~32767 ��Ӧ-327.68~327.67A   ����   �������أ�-32768~32767 ��Ӧ-327.68~327.67Nm
	* param[in]			ctrl_status��0�������������ƣ�1�����ؿ���ģʽ��2���������ƶ�����ģʽ(���������Զ�����)��3���ܺ��ƶ�����ģʽ(���������Զ�����)��4�������ƶ�����ģʽ(���������Զ�����)
	* param[in]			ack_status�� ���ķ���״̬��0��������  1�����ر�������1  2�����ر�������2  3�����ر�������3
  * postscript		�ʴ�ͨ��ģʽ�µĵ������ָ���ܷ���1~3���ֱ������ͣ���ʽ���ֲᡰ5 �ʴ�ģʽ�������ġ�		*/
void set_motor4310_cur_tor(uint16_t motor_id,int16_t cur_tor,uint8_t ctrl_status,uint8_t ack_status)
{
	/*(1)	���ķ���״ֻ̬��0��3������ģʽֻ��0��7�����򷵻�*/
	if(ack_status>3) 	return;
	if(ctrl_status>7)	return;
	
	/*(2)	���ctrl_status��=0���Ͳ��ǵ������ƣ��޷�һ������*/
	if(ctrl_status) //����Ť�ؿ���ģʽ���ƶ�ģʽ
	{
		if(cur_tor>3000) cur_tor=3000;
		else if(cur_tor<-3000) cur_tor=-3000;
	}
	
	/*(3)	���ctrl_status==0,��ʾ�ǵ������ƣ��޷�һ�µ���*/
	else
	{
		if(cur_tor>2000) cur_tor=2000;
		else if(cur_tor<-2000) cur_tor=-2000;
	}
	
	/*(4)	can����*/
	CAN_TxHeaderTypeDef motor_cur_setting_tx_message;
	uint8_t motor_cur_setting_can_send_data[3];
	uint32_t send_mail_box;
	motor_cur_setting_tx_message.StdId = motor_id;
	motor_cur_setting_tx_message.IDE = CAN_ID_STD;//CAN_ID_EXT
	motor_cur_setting_tx_message.RTR = CAN_RTR_DATA;
	motor_cur_setting_tx_message.DLC = 3;
	
  motor_cur_setting_can_send_data[0]=0x60|ctrl_status<<2|ack_status;
	motor_cur_setting_can_send_data[1]=cur_tor>>8;
	motor_cur_setting_can_send_data[2]=cur_tor&0xff;
	
  HAL_CAN_AddTxMessage(&MOTOR4310_CAN,&motor_cur_setting_tx_message,motor_cur_setting_can_send_data,&send_mail_box);
}
/********************[3.2.2] �������ָ��(�ʴ�ͨ��ģʽ)(��������)***********************/
/**	[3.2.2.1]			�ʴ�ͨ��ģʽ
  * brief         �������ָ��(�ʴ�ͨ��ģʽ)(���ٶ�����)
  * param[in]			motor_id:1~0x7FE
  * param[in]			acc:    ���ٶ���ֵ��0~2000 ��Ӧ 0~20rad/s���ڲ����޷������ٶȼ�С�����л����ͣ����ǹ�С�׳�����ϵͳĬ��ֵΪ 2000
	* param[in]			ack_status�� ���ķ���״̬��		0��������		1�����ر�������4(����4)	2�������أ�������������Ӧ������ʧ��	3�������أ�������������Ӧ������ʧ��
  * postscript		�ʴ�ͨ��ģʽ�µĵ������ָ��ֻ�ܷ��ر�������4����ʽ���ֲᡰ5 �ʴ�ģʽ�������ġ�		*/
void set_motor4310_acceleration(uint16_t motor_id,uint16_t acc,uint8_t ack_status)
{
	if(ack_status>2) return;
	if(acc>2000) acc=2000;
	
	
	CAN_TxHeaderTypeDef motor_acc_setting_tx_message;
	uint8_t motor_acc_setting_can_send_data[8];
	uint32_t send_mail_box;
	motor_acc_setting_tx_message.IDE = CAN_ID_STD;//CAN_ID_EXT
	motor_acc_setting_tx_message.StdId = motor_id;
	motor_acc_setting_tx_message.RTR = CAN_RTR_DATA;
	motor_acc_setting_tx_message.DLC = 4;
	
  motor_acc_setting_can_send_data[0]=0xC0|ack_status;
	motor_acc_setting_can_send_data[1]=0x01;
	motor_acc_setting_can_send_data[2]=acc>>8;
	motor_acc_setting_can_send_data[3]=acc&0xff;
	
  HAL_CAN_AddTxMessage(&MOTOR4310_CAN,&motor_acc_setting_tx_message,motor_acc_setting_can_send_data,&send_mail_box);
}

/**	[3.2.2.2]			�ʴ�ͨ��ģʽ
  * brief         �������ָ��(�ʴ�ͨ��ģʽ)(�����۲�������Ŷ�����ϵ������)
  * param[in]			motor_id:1~0x7FE
  * param[in]			linkage���ٶȻ���Kp���Ŷ�����ϵ����0~10000 ��Ӧ vesc tool �е� 0~1��Ĭ��ֵ 60����Ӧ 0.006
  * param[in]			speedKI���ٶȻ���KI�д����۲����棻0~10000 ��Ӧ vesc tool �е� 0~1��Ĭ��ֵ 1000����Ӧ 0.1��
	* param[in]			ack_status�� ���ķ���״̬��		0��������		1�����ر�������4(����4)	2�������أ�������������Ӧ������ʧ��	3�������أ�������������Ӧ������ʧ��
  * postscript		�ʴ�ͨ��ģʽ�µĵ������ָ��ֻ�ܷ��ر�������4����ʽ���ֲᡰ5 �ʴ�ģʽ�������ġ�		*/
void set_motor4310_linkage_speedKI(uint16_t motor_id,uint16_t linkage,uint16_t speedKI,uint8_t ack_status)
{
	if(ack_status>2) 	return;
	if(linkage>10000) linkage=10000;
	if(speedKI>10000) speedKI=10000;
		
	CAN_TxHeaderTypeDef motor_spdKP_spdKI_setting_tx_message;
	uint8_t motor_spdKP_spdKI_setting_can_send_data[6];
	uint32_t send_mail_box;
	motor_spdKP_spdKI_setting_tx_message.StdId = motor_id;
	motor_spdKP_spdKI_setting_tx_message.RTR = CAN_RTR_DATA;
	motor_spdKP_spdKI_setting_tx_message.IDE = CAN_ID_STD;//CAN_ID_EXT
	motor_spdKP_spdKI_setting_tx_message.DLC = 6;

  motor_spdKP_spdKI_setting_can_send_data[0]=0xC0|ack_status;
	motor_spdKP_spdKI_setting_can_send_data[1]=0x02;
	motor_spdKP_spdKI_setting_can_send_data[2]=linkage>>8;
	motor_spdKP_spdKI_setting_can_send_data[3]=linkage&0xff;
	motor_spdKP_spdKI_setting_can_send_data[4]=speedKI>>8;
	motor_spdKP_spdKI_setting_can_send_data[5]=speedKI&0xff;
	
	HAL_CAN_AddTxMessage(&MOTOR4310_CAN,&motor_spdKP_spdKI_setting_tx_message,motor_spdKP_spdKI_setting_can_send_data,&send_mail_box);
}

/**	[3.2.2.3]			�ʴ�ͨ��ģʽ
  * brief         �������ָ��(�ʴ�ͨ��ģʽ)(�����������ϵ��������ϵ����KD������)
  * param[in]			motor_id:1~0x7FE
  * param[in]			fdbKP�� λ�û���Kp��Ϊ��������ϵ���������������ϵ����0~10000 ��Ӧ vesc tool �е� 0~1��Ĭ��ֵ60����Ӧ0.006
  * param[in]			fdbKD��	λ�û���Kd��Ϊ����ϵ����0~10000 ��Ӧ vesc tool �е� 0~1��Ĭ��ֵ 0
	* param[in]			ack_status�� ���ķ���״̬��		0��������		1�����ر�������4(����4)	2�������أ�������������Ӧ������ʧ��	3�������أ�������������Ӧ������ʧ��
  * postscript		�ʴ�ͨ��ģʽ�µĵ������ָ��ֻ�ܷ��ر�������4����ʽ���ֲᡰ5 �ʴ�ģʽ�������ġ�		*/
void set_motor4310_feedback_tKP_KD(uint16_t motor_id,uint16_t fdbKP,uint16_t fdbKD,uint8_t ack_status)
{
	if(ack_status>2) 	return;
	if(fdbKP>10000) 	fdbKP=10000;
	if(fdbKD>10000) 	fdbKD=10000;
		
	CAN_TxHeaderTypeDef motor_posKP_posKD_setting_tx_message;
	uint8_t motor_posKP_posKD_setting_can_send_data[6];
	uint32_t send_mail_box;
	motor_posKP_posKD_setting_tx_message.StdId = motor_id;
	motor_posKP_posKD_setting_tx_message.RTR = CAN_RTR_DATA;
	motor_posKP_posKD_setting_tx_message.IDE = CAN_ID_STD;//CAN_ID_EXT
	motor_posKP_posKD_setting_tx_message.DLC = 6;

  motor_posKP_posKD_setting_can_send_data[0]=0xC0|ack_status;
	motor_posKP_posKD_setting_can_send_data[1]=0x03;
	motor_posKP_posKD_setting_can_send_data[2]=fdbKP>>8;
	motor_posKP_posKD_setting_can_send_data[3]=fdbKP&0xff;
	motor_posKP_posKD_setting_can_send_data[4]=fdbKD>>8;
	motor_posKP_posKD_setting_can_send_data[5]=fdbKD&0xff;
	
  HAL_CAN_AddTxMessage(&MOTOR4310_CAN,&motor_posKP_posKD_setting_tx_message,motor_posKP_posKD_setting_can_send_data,&send_mail_box);
}

/********************[3.2.3] �������ָ��(�ʴ�ͨ��ģʽ)(������ѯ)***********************/
/**	[3.2.3.1]			�ʴ�ͨ��ģʽ
  * brief         �������ָ��(�ʴ�ͨ��ģʽ)(���Ʋ�����ѯ)
  * param[in]			motor_id:1~0x7FE
  * param[in]			param_cmd�� ��ѯ����  0��������Ŀǰ��Ч��1����ѯ��ǰλ�ã�2����ѯ��ǰ�ٶȣ�3����ѯ��ǰ������4����ѯ��ǰ����
																				5����ѯ��ǰ���ٶȣ�6����ѯ��ǰ�����۲����棻7����ѯ��ǰ�Ŷ�����ϵ����8����ѯ��������ϵ����9����ѯ����ϵ��
  * postscript		�ʴ�ͨ��ģʽ�µĵ����ѯָ��ֻ�ܷ��ر�������5����ʽ���ֲᡰ5 �ʴ�ģʽ�������ġ�		*/
void get_motor4310_parameter(uint16_t motor_id,uint8_t param_cmd)
{
	
	CAN_TxHeaderTypeDef motor_parameter_getting_tx_message;
	uint8_t motor_parameter_getting_can_send_data[2];
	uint32_t send_mail_box;
	motor_parameter_getting_tx_message.StdId = motor_id;
	motor_parameter_getting_tx_message.RTR = CAN_RTR_DATA;
	motor_parameter_getting_tx_message.IDE = CAN_ID_STD;//CAN_ID_EXT
	motor_parameter_getting_tx_message.DLC = 2;
	
  motor_parameter_getting_can_send_data[0]=0xE0;
	motor_parameter_getting_can_send_data[1]=param_cmd;
	
  HAL_CAN_AddTxMessage(&MOTOR4310_CAN,&motor_parameter_getting_tx_message,motor_parameter_getting_can_send_data,&send_mail_box);
}

/*****************************************[4] ������ر��Ľ���*****************************************************************/

motor4310_feedback_t motor4310_feedback;
uint16_t motor_id_check=0;

void get_motor4310_data(CAN_RxHeaderTypeDef *RxMessage,uint8_t RxData[],uint8_t comm_mode)
{
	uint8_t motor_id_t=0;
	uint8_t ack_status=0;
	int pos_int=0;            
	int spd_int=0;
	int cur_int=0;
	
	
	/*(1)		[1]�������ָ���[2]�����ѯָ��е�����ʽ�ı��ķ��أ����ر��ĵı�ʶ������0x7FF*/
	if(RxMessage->StdId==0x7FF)
	{	
		/*(1.1)	ͨ��Byte3λ���ж��ǲ��ǵ�����صı���*/
		/*(1.1.1)	���Byte3λ����0x01��ֱ�ӷ���*/
		if(RxData[2]!=0x01)//Byte2 λ��0x01��ʾ�ǵ�����ظ���Ƭ���ı��ģ�0x00��ʾ�ǵ�Ƭ����������͵�ָ����ա�2.1���ͨ��ģʽ����ָ�
			return;	
		
		/*(1.2)	ͨ��Byte0��Byte1λ���ж��ǲ��ǡ���ѯCANͨ��ID��*/
		/*(1.2.1)���Byte0��Byte1λ����0xFF����ʾ����ѯCANͨ��ID����ѯ�ɹ�*/
		if((RxData[0]==0xff)&&(RxData[1]==0xFF))
		{
			motor4310_feedback.motor_id=RxData[3]<<8|RxData[4];//��3���ֽں͵��ĸ��ֽڷֱ�洢���ID�ĸߵ�8λ
			motor4310_feedback.motor_fbd=0x01;
		}
		/*(1.2.2)���Byte0��Byte1λ����0x80����ʾ����ѯCANͨ��ID����ѯʧ��*/
		else if((RxData[0]==0x80)&&(RxData[1]==0x80))//inquire failed
		{
			motor4310_feedback.motor_id=0;
			motor4310_feedback.motor_fbd=0x80;
		}
		
		/*(1.3)	ͨ��Byte0��Byte1λ���ж��ǲ��ǡ����CANͨ��ID����ָ�*/
		/*(1.3.1)���Byte0��Byte1λ����0x7F����ʾ�����CANͨ��ID����ָ����óɹ���ʧ�ܵĻ���Ӧ��*/
		else if((RxData[0]==0x7F)&&(RxData[1]==0x7F))//reset ID succeed
		{
			motor4310_feedback.motor_id=1;
			motor4310_feedback.motor_fbd=0x05;
		}
		/*(1.4)	ʣ�µ�ָ�� ���ر��ĵ�Byte0��Byte1λ�ֱ��ǵ��ID�ĸߵ�8λ��������ȡ�������ID��Ϣ*/
		else
		{
			motor4310_feedback.motor_id=RxData[0]<<8|RxData[1];
			motor4310_feedback.motor_fbd=RxData[3];
		}
	}
	
	/*(2)		�����[3]�������ָ��(�Զ�����ģʽ)���صı��ģ��ڵ��ú�����ʱ�����ڲ���comm_mode��Ϊ0x01*/
		else if(comm_mode==0x01)//��ʾ�Զ�����ģʽautomatic feedback mode
	{
		motor_id_t=RxMessage->StdId-0x205;
		motor4310_data[motor_id_t].angle_actual_int=(uint16_t)(RxData[0]<<8|RxData[1]);//�����λ��
		motor4310_data[motor_id_t].speed_actual_int=(int16_t)(RxData[2]<<8|RxData[3]);//�����ת��
		motor4310_data[motor_id_t].current_actual_int=(RxData[4]<<8|RxData[5]);//�ҿ�˵����Ӧ����ʵ��ת�ذɣ�����ת�غ͵���Ӧ��Ҳ�ǹ�����
		motor4310_data[motor_id_t].temperature=RxData[6];//����¶�
		motor4310_data[motor_id_t].error=RxData[7];//������Ϣ
	}
	
	/*(2)		�����[3]�������ָ��(�ʴ�ͨ��ģʽ)���صı��ģ��ڵ��ú�����ʱ�����ڲ���comm_mode��Ϊ0x00*/
	else if(comm_mode==0x00)//��ʾ�ʴ�ģʽResponse mode
	{
																				
		ack_status=RxData[0]>>5;			//��һ���ֽ�5��7λ��ʾ�������ͣ�1-3��ʾ����ָ��ر��ĵ��������ͣ�4��ʾ����ָ��ر��ģ�5��ʾ��ѯָ��ر���
		motor_id_t=RxMessage->StdId-1;
		motor_id_check=RxMessage->StdId;
		motor4310_data[motor_id_t].motor_id=motor_id_t;
		motor4310_data[motor_id_t].error=RxData[0]&0x1F;//��һ���ֽ�0��4λ��ʾ���������Ϣ
		
		/*(2.1)	�����[3]�������ָ��(�ʴ�ͨ��ģʽ)(��ͬ����ģʽ)����ָ����صı�������1*/
		if(ack_status==1)
		{
			/*����int�����ݣ�λ�á�ת�١�����*/
			pos_int=RxData[1]<<8|RxData[2];//�����λ��
			spd_int=RxData[3]<<4|(RxData[4]&0xF0)>>4;//�����ת��
			cur_int=(RxData[4]&0x0F)<<8|RxData[5];//��������
			/*ת��һ���������� ���� ������Ϣ�Ľṹ�壬˳����¶���ϢҲ�ó��� */
			motor4310_data[motor_id_t].angle_actual_rad=uint_to_float(pos_int,POS_MIN,POS_MAX,16);
			motor4310_data[motor_id_t].speed_actual_rad=uint_to_float(spd_int,SPD_MIN,SPD_MAX,12);
			motor4310_data[motor_id_t].current_actual_float=uint_to_float(cur_int,I_MIN,I_MAX,12);
			motor4310_data[motor_id_t].temperature=(RxData[6]-50)/2;
		}
		
		/*(2.2)	�����[3]�������ָ��(�ʴ�ͨ��ģʽ)(��ͬ����ģʽ)����ָ����صı�������2*/		
		else if(ack_status==2)
		{
			/*�ȸ��������巽�������������ת��*/
			rv_type_convert.buf[0]=RxData[4];
			rv_type_convert.buf[1]=RxData[3];
			rv_type_convert.buf[2]=RxData[2];
			rv_type_convert.buf[3]=RxData[1];
			
			motor4310_data[motor_id_t].angle_actual_float=rv_type_convert.to_float;//�����λ��
			motor4310_data[motor_id_t].current_actual_int=RxData[5]<<8|RxData[6];//��������
			motor4310_data[motor_id_t].temperature=(RxData[7]-50)/2;//����¶�
			motor4310_data[motor_id_t].current_actual_float=motor4310_data[motor_id_t].current_actual_int/100.0f;
		}
		
		/*(2.3)	�����[3]�������ָ��(�ʴ�ͨ��ģʽ)(��ͬ����ģʽ)����ָ����صı�������3*/		
		else if(ack_status==3)//������ʴ�ͨ��ģʽ���ر�������3���ǿ���ָ��صı�������֮һ  response frame 3
		{
			rv_type_convert.buf[0]=RxData[4];
			rv_type_convert.buf[1]=RxData[3];
			rv_type_convert.buf[2]=RxData[2];
			rv_type_convert.buf[3]=RxData[1];
			
			motor4310_data[motor_id_t].speed_actual_float=rv_type_convert.to_float;//������ٶ�
			motor4310_data[motor_id_t].current_actual_int=RxData[5]<<8|RxData[6];//��������
			motor4310_data[motor_id_t].temperature=(RxData[7]-50)/2;//����¶�
			motor4310_data[motor_id_t].current_actual_float=motor4310_data[motor_id_t].current_actual_int/100.0f;
		}
		
		/*(2.4)	�����[3]�������ָ��(�ʴ�ͨ��ģʽ)(��������)����ָ����صı�������4*/
		else if(ack_status==4)//������ʴ�ͨ��ģʽ���ر�������4��������ָ��صı������� response frame 4
		{
			if(RxMessage->DLC!=3)	return;
			motor4310_feedback.INS_code=RxData[1];//���ô���
			motor4310_feedback.motor_fbd=RxData[2];//����״̬��0��ʾʧ�ܣ�1��ʾ�ɹ�
		}
		
		/*(2.5)	�����[3]�������ָ��(�ʴ�ͨ��ģʽ)(������ѯ)��ѯָ����صı�������5*/
		else if(ack_status==5)//������ʴ�ͨ��ģʽ���ر�������5���ǲ�ѯָ��صı������� response frame 5
		{
			/*(2.5.1)	��ȡһ�²�ѯ���룬���뷶Χ1-9*/
			motor4310_feedback.INS_code=RxData[1];
			
			/*(2.5.2) ���ݲ�ѯ��������������Ӧ��ֵ*/
			/*(2.5.2.1)	1��λ������*/
			if(motor4310_feedback.INS_code==1&RxMessage->DLC==6)
			{
				rv_type_convert.buf[0]=RxData[5];
				rv_type_convert.buf[1]=RxData[4];
				rv_type_convert.buf[2]=RxData[3];
				rv_type_convert.buf[3]=RxData[2];
				
				motor4310_data[motor_id_t].angle_actual_float=rv_type_convert.to_float;//��ǰλ�ã�float�ͣ�
			}
			/*(2.5.2.2)	2���ٶ�����*/
			else if(motor4310_feedback.INS_code==2&RxMessage->DLC==6)//
			{
				rv_type_convert.buf[0]=RxData[5];
				rv_type_convert.buf[1]=RxData[4];
				rv_type_convert.buf[2]=RxData[3];
				rv_type_convert.buf[3]=RxData[2];
				
				motor4310_data[motor_id_t].speed_actual_float=rv_type_convert.to_float;//��ǰ�ٶȣ�float�ͣ�
			}
			/*(2.5.2.3)	3����������*/
			else if(motor4310_feedback.INS_code==3&RxMessage->DLC==6)//
			{
				rv_type_convert.buf[0]=RxData[5];
				rv_type_convert.buf[1]=RxData[4];
				rv_type_convert.buf[2]=RxData[3];
				rv_type_convert.buf[3]=RxData[2];
				motor4310_data[motor_id_t].current_actual_float=rv_type_convert.to_float;//��ǰ������float�ͣ�
			}
			/*(2.5.2.4)	4����������*/
			else if(motor4310_feedback.INS_code==4&RxMessage->DLC==6)//get power
			{
				rv_type_convert.buf[0]=RxData[5];
				rv_type_convert.buf[1]=RxData[4];
				rv_type_convert.buf[2]=RxData[3];
				rv_type_convert.buf[3]=RxData[2];
				motor4310_data[motor_id_t].power=rv_type_convert.to_float;//��ǰ���ʣ�float�ͣ�
			}
			/*(2.5.2.5)	5�����ٶ�����*/
			else if(motor4310_feedback.INS_code==5&RxMessage->DLC==4)//get acceleration
			{
				motor4310_data[motor_id_t].acceleration=RxData[2]<<8|RxData[3];//��ǰ���ٶȣ�uint16�ͣ�
			}
			/*(2.5.2.6)	6�������۲�����*/
			else if(motor4310_feedback.INS_code==6&RxMessage->DLC==4)//get linkage_KP
			{
				motor4310_data[motor_id_t].linkage_KP=RxData[2]<<8|RxData[3];//��ǰ�����۲����棨uint16�ͣ�
			}
			/*(2.5.2.7)	7���Ŷ�����ϵ��*/
			else if(motor4310_feedback.INS_code==7&RxMessage->DLC==4)//get speed_KI
			{
				motor4310_data[motor_id_t].speed_KI=RxData[2]<<8|RxData[3];//��ǰ�Ŷ�����ϵ����uint16�ͣ�
			}
			/*(2.5.2.8)	8��������������*/
			else if(motor4310_feedback.INS_code==8&RxMessage->DLC==4)//get feedback_KP
			{
				motor4310_data[motor_id_t].feedback_KP=RxData[2]<<8|RxData[3];//��ǰ�����������棨uint16�ͣ�
			}
			/*(2.5.2.9)	9������ϵ��*/
			else if(motor4310_feedback.INS_code==9&RxMessage->DLC==4)//get feedback_KD
			{
				motor4310_data[motor_id_t].feedback_KD=RxData[2]<<8|RxData[3];//��ǰ����ϵ����uint16�ͣ�
			}

		}
	}
	
}




