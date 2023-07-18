#include "aim_motor.h"
#include "motor_can.h"
#include "pid.h"
#include "remote_control.h"
#include "shoot_task.h"
#include "arm_math.h"

motor_data_t 		aim_motor_data;		
pid_type_def		aim_motor_encoder_angle_pid;	
pid_type_def 		aim_motor_encoder_speed_pid;

fp32 aim_motor_encoder_speed_pid_Kp				=1.0f;
fp32 aim_motor_encoder_speed_pid_Ki				=0.1f;
fp32 aim_motor_encoder_speed_pid_Kd				=0.0f;
fp32 aim_motor_encoder_speed_pid_max_out	=2000.0f;
fp32 aim_motor_encoder_speed_pid_max_iout	=1000.0f;
fp32 aim_motor_encoder_speed_PID[3]; 

fp32 aim_motor_encoder_angle_pid_Kp				=0.15f;
fp32 aim_motor_encoder_angle_pid_Ki				=0.0f;
fp32 aim_motor_encoder_angle_pid_Kd				=1.8f;
fp32 aim_motor_encoder_angle_pid_max_out	=1000.0f;
fp32 aim_motor_encoder_angle_pid_max_iout	=0.0f;
fp32 aim_motor_encoder_angle_PID[3]; 

double aim_motor_angle;
int16_t KEY_R;
int16_t KEY_R_last;
int16_t KEY_F;
int16_t KEY_F_last;
int16_t KEY_G;
int16_t KEY_G_last;

/*****2006�����궨��*****/
#define Delta_2006_Ecd_After_One_Round 8192			//3508תһȦ֮���������ֵ�ı���
#define Reduction_Ratio_of_2006_Motor 36		//3508����ļ��ٱ�(�궨������ֲ������㣬���治�ӷֺţ���Ȼ�˷�����ʱ�ᱻ����ָ��)
#define Delta_Total_2006_Ecd_After_One_Round Delta_2006_Ecd_After_One_Round*Reduction_Ratio_of_2006_Motor//�óɺ궨����⾫����ʧ

RC_ctrl_t rc_ctrl_last3;
uint8_t aim_on_off_flag=0;//�ϵ�֮ǰҪ��λ��������λ��
int64_t aim_motor_offset_total_ecd=0;			 //�ϲ����̸�λƫ���ܱ������Ƕȣ���λƫ���ܱ������Ƕȣ�Ҳ���Ǵ�����ܱ������ǶȻ�����ת�̶��Ƕ�

uint8_t aim_motor_start_reset_locked_rotor_flag;
uint8_t aim_motor_start_reset_locked_rotor_flag_last;
uint16_t aim_motor_start_reset_locked_rotor_cnt;

uint8_t aim_motor_guan_jing_locked_rotor_flag;
uint16_t aim_motor_guan_jing_locked_rotor_cnt;
void Aim_Motor_Control()
{
	
	if(aim_motor_start_reset_locked_rotor_flag==0)//���û���ϵ��ת��λ
	{
		/*(1)	�ظ�λ����ת*/
		aim_motor_data.target_speed_rpm=-200;
		calculate_aim_motor_current_with_target_total_speed();
		
		/*(2)	��ת���*/
		locked_rotor_detect(aim_motor_data.give_current,-700,20,&aim_motor_start_reset_locked_rotor_flag,&aim_motor_start_reset_locked_rotor_cnt);//���������ת���
		/*(3)	�ռ�⵽��ת*/
		if(aim_motor_start_reset_locked_rotor_flag_last!=1 && aim_motor_start_reset_locked_rotor_flag==1)
		{
			aim_motor_offset_total_ecd=aim_motor_data.total_ecd;//��ƫ�ýǶȣ��Ժ�ÿ��ת��Ҫ�Ե�ǰλ��Ϊ����ת�̶��Ƕ�
			aim_motor_data.target_total_ecd=aim_motor_data.total_ecd+Delta_Total_2006_Ecd_After_One_Round*2.0f/360.0f;//�����ȶ��ڵ�ǰ�Ƕ�
		}
		aim_motor_start_reset_locked_rotor_flag_last=aim_motor_start_reset_locked_rotor_flag;
	}
	
	else//�����ת��λ����
	{
			KEY_R=((rc_ctrl.key.v & KEY_PRESSED_OFFSET_R) >>8);
			KEY_F=((rc_ctrl.key.v & KEY_PRESSED_OFFSET_F) >>9);
			KEY_G=((rc_ctrl.key.v & KEY_PRESSED_OFFSET_G) >>10);
			
			if(KEY_F!=1 && KEY_G!=1)//���û�����ֶ�������F��G
			{
				if(KEY_R_last!=1 && KEY_R==1)//R�����ؾ�
					aim_on_off_flag=!aim_on_off_flag;

				if(aim_on_off_flag==1)
					aim_motor_data.target_total_ecd=+Delta_Total_2006_Ecd_After_One_Round*200.0f/360.0f+aim_motor_offset_total_ecd;
				if(aim_on_off_flag==0)
					aim_motor_data.target_total_ecd=aim_motor_offset_total_ecd;
				
				calculate_aim_motor_current_with_target_total_angle();
			}
			
			if(KEY_F==1)//�������F�ֶ�����
			{
				aim_motor_data.target_speed_rpm=-200;
				calculate_aim_motor_current_with_target_total_speed();
			}
			
			if(KEY_G==1)//�������G�ֶ�����
			{
				aim_motor_data.target_speed_rpm=200;
				calculate_aim_motor_current_with_target_total_speed();
			}
			
			if(	( (KEY_F_last!=0) && (KEY_F==0) ) || (	(KEY_G_last!=0) && (KEY_G==0)	)		)//����ɿ�����F��G
			{
				//��Ҫ���ݿ��ؾ�flag��״̬ȷ��ƫ�ýǶȴ�С����Ϊ����ɿ���������ִ����֮����Ż�ִ�С�û�����ֶ�������F��G���ĳ��򣬸���Ŀ��Ƕȡ�����Ŀ�����ɿ��������þ����ڵ�ǰ�Ƕ�
				if(aim_on_off_flag==1)//�������״̬���ɿ�F��G����û�����ֶ�������F��G���ĳ��������Ŀ��Ƕ�Ϊƫ�ýǶȼ�180�ȣ�Ϊ���ȶ����ڵ�ǰ�Ƕȣ���Ҫ��ƫ�ýǶ��������ȼ�ȥ180��
					aim_motor_offset_total_ecd=aim_motor_data.total_ecd-Delta_Total_2006_Ecd_After_One_Round*200.0f/360.0f;
				
				if(aim_on_off_flag==0)//����ؾ�״̬���ɿ�F��G��������������Ŀ��Ƕ�Ϊƫ�ýǶȣ���������Ͱ�ƫ�ýǶ�����Ϊ��ǰ�Ƕ�
					aim_motor_offset_total_ecd=aim_motor_data.total_ecd;
			}
			
			
			locked_rotor_detect(aim_motor_data.give_current,-1800,50,&aim_motor_guan_jing_locked_rotor_flag,&aim_motor_guan_jing_locked_rotor_cnt);//���������ת���
			
			if(aim_motor_guan_jing_locked_rotor_flag==1)//�����⵽�ؾ�ʱ������ת
			{//Ҳ��Ҫ���ݿ��ؾ�flag��״̬ȷ��ƫ�ýǶȴ�С
				if(aim_on_off_flag==1)
					aim_motor_offset_total_ecd=aim_motor_data.total_ecd-Delta_Total_2006_Ecd_After_One_Round*200.0f/360.0f;
				
				if(aim_on_off_flag==0)
					aim_motor_offset_total_ecd=aim_motor_data.total_ecd;
				
				aim_motor_guan_jing_locked_rotor_flag=0;//��־λ����
			}
			
			
	}
	
	
	
	KEY_R_last=KEY_R;
	KEY_F_last=KEY_F;
	KEY_G_last=KEY_G;
	rc_ctrl_last3=rc_ctrl;
}


void calculate_aim_motor_current_with_target_total_angle()
{
		aim_motor_data.target_speed_rpm=PID_calc(	&aim_motor_encoder_angle_pid,	aim_motor_data.total_ecd, aim_motor_data.target_total_ecd);

		aim_motor_data.target_current= PID_calc(	&aim_motor_encoder_speed_pid,	aim_motor_data.speed_rpm,	aim_motor_data.target_speed_rpm);
}

void calculate_aim_motor_current_with_target_total_speed()
{
		aim_motor_data.target_current= PID_calc(	&aim_motor_encoder_speed_pid,	aim_motor_data.speed_rpm,	aim_motor_data.target_speed_rpm);
}

void aim_motor_init()
{
	aim_motor_encoder_angle_PID[0]	=	aim_motor_encoder_angle_pid_Kp;
	aim_motor_encoder_angle_PID[1]	=	aim_motor_encoder_angle_pid_Ki;
	aim_motor_encoder_angle_PID[2]	=	aim_motor_encoder_angle_pid_Kd;
	PID_init(	&aim_motor_encoder_angle_pid,	PID_POSITION,	aim_motor_encoder_angle_PID,	aim_motor_encoder_angle_pid_max_out,	aim_motor_encoder_angle_pid_max_iout);
	
	aim_motor_encoder_speed_PID[0]	=	aim_motor_encoder_speed_pid_Kp;
	aim_motor_encoder_speed_PID[1]	=	aim_motor_encoder_speed_pid_Ki;
	aim_motor_encoder_speed_PID[2]	=	aim_motor_encoder_speed_pid_Kd;
	PID_init(	&aim_motor_encoder_speed_pid,	PID_POSITION,	aim_motor_encoder_speed_PID,	aim_motor_encoder_speed_pid_max_out,	aim_motor_encoder_speed_pid_max_iout);
	
}
