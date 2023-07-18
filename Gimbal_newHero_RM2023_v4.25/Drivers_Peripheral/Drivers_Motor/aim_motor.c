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

/*****2006参数宏定义*****/
#define Delta_2006_Ecd_After_One_Round 8192			//3508转一圈之后编码器数值改变量
#define Reduction_Ratio_of_2006_Motor 36		//3508电机的减速比(宏定义的数字参与运算，后面不加分号，不然乘法运算时会被当成指针)
#define Delta_Total_2006_Ecd_After_One_Round Delta_2006_Ecd_After_One_Round*Reduction_Ratio_of_2006_Motor//用成宏定义避免精度损失

RC_ctrl_t rc_ctrl_last3;
uint8_t aim_on_off_flag=0;//上电之前要复位到开镜的位置
int64_t aim_motor_offset_total_ecd=0;			 //上拨弹盘复位偏置总编码器角度；复位偏置总编码器角度，也就是从这个总编码器角度基础上转固定角度

uint8_t aim_motor_start_reset_locked_rotor_flag;
uint8_t aim_motor_start_reset_locked_rotor_flag_last;
uint16_t aim_motor_start_reset_locked_rotor_cnt;

uint8_t aim_motor_guan_jing_locked_rotor_flag;
uint16_t aim_motor_guan_jing_locked_rotor_cnt;
void Aim_Motor_Control()
{
	
	if(aim_motor_start_reset_locked_rotor_flag==0)//如果没有上电堵转复位
	{
		/*(1)	沿复位方向转*/
		aim_motor_data.target_speed_rpm=-200;
		calculate_aim_motor_current_with_target_total_speed();
		
		/*(2)	堵转检测*/
		locked_rotor_detect(aim_motor_data.give_current,-700,20,&aim_motor_start_reset_locked_rotor_flag,&aim_motor_start_reset_locked_rotor_cnt);//倍镜电机堵转检测
		/*(3)	刚检测到堵转*/
		if(aim_motor_start_reset_locked_rotor_flag_last!=1 && aim_motor_start_reset_locked_rotor_flag==1)
		{
			aim_motor_offset_total_ecd=aim_motor_data.total_ecd;//改偏置角度，以后每次转都要以当前位置为基点转固定角度
			aim_motor_data.target_total_ecd=aim_motor_data.total_ecd+Delta_Total_2006_Ecd_After_One_Round*2.0f/360.0f;//让它先定在当前角度
		}
		aim_motor_start_reset_locked_rotor_flag_last=aim_motor_start_reset_locked_rotor_flag;
	}
	
	else//如果堵转复位完了
	{
			KEY_R=((rc_ctrl.key.v & KEY_PRESSED_OFFSET_R) >>8);
			KEY_F=((rc_ctrl.key.v & KEY_PRESSED_OFFSET_F) >>9);
			KEY_G=((rc_ctrl.key.v & KEY_PRESSED_OFFSET_G) >>10);
			
			if(KEY_F!=1 && KEY_G!=1)//如果没按下手动调镜的F、G
			{
				if(KEY_R_last!=1 && KEY_R==1)//R键开关镜
					aim_on_off_flag=!aim_on_off_flag;

				if(aim_on_off_flag==1)
					aim_motor_data.target_total_ecd=+Delta_Total_2006_Ecd_After_One_Round*200.0f/360.0f+aim_motor_offset_total_ecd;
				if(aim_on_off_flag==0)
					aim_motor_data.target_total_ecd=aim_motor_offset_total_ecd;
				
				calculate_aim_motor_current_with_target_total_angle();
			}
			
			if(KEY_F==1)//如果按下F手动调镜
			{
				aim_motor_data.target_speed_rpm=-200;
				calculate_aim_motor_current_with_target_total_speed();
			}
			
			if(KEY_G==1)//如果按下G手动调镜
			{
				aim_motor_data.target_speed_rpm=200;
				calculate_aim_motor_current_with_target_total_speed();
			}
			
			if(	( (KEY_F_last!=0) && (KEY_F==0) ) || (	(KEY_G_last!=0) && (KEY_G==0)	)		)//如果松开按键F、G
			{
				//需要根据开关镜flag的状态确定偏置角度大小，因为这个松开按键程序执行完之后接着会执行“没按下手动调镜的F、G”的程序，更新目标角度。最终目的是松开按键后让镜稳在当前角度
				if(aim_on_off_flag==1)//如果开镜状态下松开F或G，“没按下手动调镜的F、G”的程序会设置目标角度为偏置角度加180度，为了稳定了在当前角度，需要将偏置角度在这里先减去180度
					aim_motor_offset_total_ecd=aim_motor_data.total_ecd-Delta_Total_2006_Ecd_After_One_Round*200.0f/360.0f;
				
				if(aim_on_off_flag==0)//如果关镜状态下松开F或G，后面程序会设置目标角度为偏置角度，所以这里就把偏置角度设置为当前角度
					aim_motor_offset_total_ecd=aim_motor_data.total_ecd;
			}
			
			
			locked_rotor_detect(aim_motor_data.give_current,-1800,50,&aim_motor_guan_jing_locked_rotor_flag,&aim_motor_guan_jing_locked_rotor_cnt);//倍镜电机堵转检测
			
			if(aim_motor_guan_jing_locked_rotor_flag==1)//如果检测到关镜时发生堵转
			{//也需要根据开关镜flag的状态确定偏置角度大小
				if(aim_on_off_flag==1)
					aim_motor_offset_total_ecd=aim_motor_data.total_ecd-Delta_Total_2006_Ecd_After_One_Round*200.0f/360.0f;
				
				if(aim_on_off_flag==0)
					aim_motor_offset_total_ecd=aim_motor_data.total_ecd;
				
				aim_motor_guan_jing_locked_rotor_flag=0;//标志位清零
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
