#include "yaw_motor.h"
#include "motor_can.h"
#include "pid.h"
#include "remote_control.h"
#include "BMI088driver.h"
#include <stdlib.h>
#include "math.h"
#include "Nmanifold_usart_task.h"
/*********************************************************数据信息和控制信息*********************************************************/
motor_data_t 		yaw_motor_data;				
pid_type_def		yaw_motor_INS_angle_pid;	
pid_type_def 		yaw_motor_INS_speed_pid;
pid_type_def 		yaw_motor_key_INS_speed_pid;
pid_type_def    yaw_motor_autoaim_INS_speed_pid;
pid_type_def    yaw_motor_autoaim_INS_angle_pid;
pid_type_def		yaw_motor_encoder_angle_pid;
pid_type_def 		yaw_motor_encoder_speed_pid;	

/*********************************************************电机控制PID参数*********************************************************/
		/*****************遥控模式*****************/
		/*yaw轴			遥控模式			陀螺仪速度环*/
		fp32 yaw_motor_INS_speed_pid_Kp				=10000.0f;
		fp32 yaw_motor_INS_speed_pid_Ki				=20.0f;
		fp32 yaw_motor_INS_speed_pid_Kd				=0.0f;
		fp32 yaw_motor_INS_speed_pid_max_out	=30000.0f;
		fp32 yaw_motor_INS_speed_pid_max_iout	=15000.0f;
		fp32 yaw_motor_INS_speed_PID[3]; 
		/*之前参数(新调的在左)：(4000,20,0.30000,15000)(40000.0f，28.0f，0.0f，30000.0f，500.0f);*/

		/*yaw轴			遥控模式			陀螺仪位置环*/
		fp32 yaw_motor_INS_angle_pid_Kp				=0.8;
		fp32 yaw_motor_INS_angle_pid_Ki				=0.0f;
		fp32 yaw_motor_INS_angle_pid_Kd				=2.0f;
		fp32 yaw_motor_INS_angle_pid_max_out	=10.0f;
		fp32 yaw_motor_INS_angle_pid_max_iout	=0.0f;
		fp32 yaw_motor_INS_angle_PID[3];
		/*之前参数：(0.05f，0.0f，2.7f，10.0f，0.0f);	(新调的在左)*/

		/************云台跟随底盘模式*****************/
		/*yaw轴		云台跟底盘模式		编码器位置环*/
		fp32 yaw_motor_encoder_angle_pid_Kp				=0.2;
		fp32 yaw_motor_encoder_angle_pid_Ki				=0.0f;
		fp32 yaw_motor_encoder_angle_pid_Kd				=1.0f;
		fp32 yaw_motor_encoder_angle_pid_max_out	=200.0f;
		fp32 yaw_motor_encoder_angle_pid_max_iout	=0.0f;
		fp32 yaw_motor_encoder_angle_PID[3];
		/*之前参数：();(0.05,0,1,200,0);(0.05,0,2.7,10,0)	(新调的在左)*/

		/*yaw轴		云台跟底盘模式		编码器速度环*/
		fp32 yaw_motor_encoder_speed_pid_Kp				=600.0f;
		fp32 yaw_motor_encoder_speed_pid_Ki				=10.0f;
		fp32 yaw_motor_encoder_speed_pid_Kd				=0.0f;
		fp32 yaw_motor_encoder_speed_pid_max_out	=20000.0f;
		fp32 yaw_motor_encoder_speed_pid_max_iout	=10000.0f;
		fp32 yaw_motor_encoder_speed_PID[3]; 
		/*之前参数：(180,3,0,20000,10000);(350,40,0,20000,10000);(40000,28,0,30000,500)		(新调的在左)*/

		/*****************自瞄模式*****************/
		/*yaw轴		自瞄模式		陀螺仪速度环*/
		fp32 yaw_motor_autoaim_INS_speed_pid_Kp				=22500.0f;
		fp32 yaw_motor_autoaim_INS_speed_pid_Ki				=80.0f;
		fp32 yaw_motor_autoaim_INS_speed_pid_Kd				=6800.0f;
		fp32 yaw_motor_autoaim_INS_speed_pid_max_out	=30000.0f;
		fp32 yaw_motor_autoaim_INS_speed_pid_max_iout	=10000.0f;
		fp32 yaw_motor_autoaim_INS_speed_PID[3]; 
		/*之前参数：(30000.0f，30.0f，0.0f，30000.0f，500.0f);	(新调的在左)*/
		
		/*yaw轴		自瞄模式		陀螺仪位置环*/
		fp32 yaw_motor_autoaim_INS_angle_pid_Kp				=0.4f;
		fp32 yaw_motor_autoaim_INS_angle_pid_Ki				=1.5f;
		fp32 yaw_motor_autoaim_INS_angle_pid_Kd				=5.0f;
		fp32 yaw_motor_autoaim_INS_angle_pid_max_out	=100.0f;
		fp32 yaw_motor_autoaim_INS_angle_pid_max_iout	=0.0f;
		fp32 yaw_motor_autoaim_INS_angle_PID[3];
		/*之前参数：(0.4,1.5,5,100,0);(1.2,2.0,3.0,100,0);(0.45,0.0,3.0,100,0)	(新调的在左)*/
		
		/*****************按键微调模式*****************/
		/*yaw轴			按键微调模式			陀螺仪速度环*/
		fp32 yaw_motor_key_INS_speed_pid_Kp				=30000.0f;
		fp32 yaw_motor_key_INS_speed_pid_Ki				=30.0f;
		fp32 yaw_motor_key_INS_speed_pid_Kd				=0.0f;
		fp32 yaw_motor_key_INS_speed_pid_max_out	=30000.0f;
		fp32 yaw_motor_key_INS_speed_pid_max_iout	=10000.0f;
		fp32 yaw_motor_key_INS_speed_PID[3]; 
/*********************************************************YAW控制程序*********************************************************/

/*****yaw轴灵敏度变量*****/
#define YAW_RC_SENS 0.01f//yaw轴遥控控制的灵敏度
//#define YAW_MOUSE_SENS 0.2f//yaw轴鼠标控制的灵敏度
float	YAW_MOUSE_SENS=0.06f;//yaw轴鼠标控制的灵敏度
float YAW_MOUSE_AIM_ON_SENS=0.02f;//开镜之后鼠标控制的灵敏度

/*****云台跟随底盘的6020偏置*****/
#define Gimbal_Follow_Chassis_Offset_Ecd	6920

/*****yaw轴的控制模式*****/
#define Yaw_RC_Mode				0								//遥控模式
#define Yaw_Autoaim_Mode	1								//自瞄模式
#define Yaw_Gimbal_Follow_Chassis_Mode 2	//底盘跟随云台模式
/*****陀螺仪数据变量*****/
extern fp32 INS_angle_deg[3];//欧拉角(degree)
extern bmi088_real_data_t bmi088_real_data;

/*****yaw轴陀螺仪变量*****/
float yaw_INS_speed;								//yaw轴陀螺仪速度
float yaw_INS_angle;								//yaw轴陀螺仪角度

float yaw_target_INS_speed;					//yaw轴陀螺仪目标速度
float yaw_target_INS_angle;					//yaw轴陀螺仪目标角度

float yaw_autoaim_target_INS_speed;	//yaw轴陀螺仪目标速度
float yaw_autoaim_target_INS_angle;	//yaw轴陀螺仪目标角度

float yaw_encoder_speed;						//yaw轴编码器速度
float yaw_encoder_angle;						//yaw轴编码器角度

float yaw_target_encoder_speed;			//yaw轴编码器目标速度
float yaw_target_encoder_angle;			//yaw轴编码器目标角度

/*****yaw三个模式的标志变量*****/
uint8_t yaw_mode_flag;//0表示遥控，1表示自瞄，2表示云台跟随底盘
uint8_t yaw_mode_flag_last;

/*****yaw是否在动的标志变量*****/
uint8_t yaw_moving_flag;//1表示yaw在动，0表示yaw不动
uint8_t yaw_moving_last_flag;

/*****yaw按键微调变量*****/
int16_t KEY_Z;
int16_t KEY_Z_last;
int16_t KEY_X;
int16_t KEY_X_last;

/*****接收的视觉自瞄变量*****/
extern AutoAim_Data_Rx AutoAim_Data_Receive;

extern uint8_t aim_on_off_flag;//开镜标志位
/**[]					
  * brief					控制yaw电机											
	* postscript							*/
uint8_t Autoaim_UI_flag;
void Yaw_Motor_Control()
{
	if(rc_ctrl.rc.s[1]==1 || rc_ctrl.rc.s[1]==3)//左拨杆拨到中间或上面，云台跟陀螺仪
	{
		if(	(rc_ctrl.mouse.press_r)	&&	(AutoAim_Data_Receive.If_Aimed==1)	)//如果右键开启自瞄并且自瞄检测到目标（连上键鼠右键开启自瞄）
//		if(	((rc_ctrl.mouse.press_r)||rc_ctrl.rc.s[1]==1)	&&	(AutoAim_Data_Receive.If_Aimed==1)	)//只用遥控器测自瞄的时候，如果右键开启自瞄或者左拨杆拨到最上面  并且自瞄检测到目标（连上键鼠右键开启自瞄）
		{Yaw_Motor_Control_in_Autoaim_Mode();Autoaim_UI_flag=1;}
		else//没开启或没识别到
		{Yaw_Motor_Control_in_RC_Mode();Autoaim_UI_flag=0;}
	}
	else if(rc_ctrl.rc.s[1]==2)
	{
		yaw_motor_data.target_current=0;
	}
	yaw_mode_flag_last=yaw_mode_flag;

}


/**	[1.1]							
  * brief         自瞄模式下控制yaw轴
	* postscript										*/
static float yaw_last;
void Yaw_Motor_Control_in_Autoaim_Mode()
{
		yaw_mode_flag=Yaw_Autoaim_Mode;//标记一下yaw的模式
	
		yaw_INS_angle=INS_angle_deg[0];//读取yaw轴陀螺仪角度
		yaw_INS_speed=bmi088_real_data.gyro[2];//读取yaw轴陀螺仪速度(整理代码之前似乎没这句话？？)
	
		yaw_autoaim_target_INS_angle=AutoAim_Data_Receive.Yaw;//读取视觉传回的目标角度
		yaw_autoaim_target_INS_angle=0.8f*yaw_last+yaw_autoaim_target_INS_angle*0.2f;
		yaw_last=yaw_autoaim_target_INS_angle;//加低通滤波
	
	
		if(yaw_autoaim_target_INS_angle-yaw_INS_angle>180)yaw_INS_angle+=360;
		if(yaw_autoaim_target_INS_angle-yaw_INS_angle<-180)yaw_INS_angle-=360;//让头走劣弧
	
		calculate_yaw_motor_current_with_autoaim_target_INS_angle();//自瞄模式位置环
}

/**	[1.2]							
  * brief         遥控模式下控制yaw轴
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
	
	
		/*(0)	读取yaw轴信息，进行模式过渡*/
	
			yaw_mode_flag=Yaw_RC_Mode;//标记一下yaw的模式
				
			KEY_Z=((rc_ctrl.key.v & KEY_PRESSED_OFFSET_Z) >> 11);
			KEY_X=((rc_ctrl.key.v & KEY_PRESSED_OFFSET_X) >> 12);//读取按键信息
	
			yaw_INS_speed =	bmi088_real_data.gyro[2];//读取yaw轴陀螺仪速度
			yaw_INS_angle=INS_angle_deg[0];//读取yaw轴陀螺仪角度

			if(yaw_mode_flag==0 && yaw_mode_flag_last!=0)//如果从其他模式切换到遥控模式，遥控模式的目标角度先设成当前角度
				yaw_target_INS_angle=INS_angle_deg[0];
			
			
		/*(1)	判断头有没有实际在动，标记为两种状态*/
			
			if(rc_ctrl.rc.ch[0]!=0 || rc_ctrl.mouse.x!=0)	yaw_moving_flag=1;
			if(rc_ctrl.rc.ch[0]==0 &&	rc_ctrl.mouse.x==0 && fabs(yaw_INS_speed)<0.5)	yaw_moving_flag=0;//根据遥控器摇杆和yaw的速度 两个条件同时判断；加上yaw速度判断可以防止yaw超调
		
		/*(2)	在头动，头即将不动，头不动三个状态下分别写动作*/
			
			if(yaw_moving_flag==1)//头动的时候设置目标速度
			{
				if(aim_on_off_flag==0)//如果没开镜，设置一个灵敏度
					yaw_target_INS_speed=-YAW_RC_SENS*rc_ctrl.rc.ch[0]-YAW_MOUSE_SENS*rc_ctrl.mouse.x;
				else //如果开镜了，鼠标灵敏度调低
					yaw_target_INS_speed=-YAW_RC_SENS*rc_ctrl.rc.ch[0]-YAW_MOUSE_AIM_ON_SENS*rc_ctrl.mouse.x;
				
				calculate_yaw_motor_current_with_target_INS_speed();
			}
			
			if(yaw_moving_last_flag==1 && yaw_moving_flag==0)//头刚停动的时候设置目标角度
				yaw_target_INS_angle=INS_angle_deg[0];
			
			if(yaw_moving_flag==0)//	头不动的时候位置环定在那儿
			{
				/*方案1：按键调整目标位置*/
				if(KEY_Z_last==0	&&	KEY_Z!=0)yaw_target_INS_angle+=0.05f;
				if(KEY_X_last==0	&&	KEY_X!=0)yaw_target_INS_angle-=0.05f;
				
//				/*方案2：按键设置目标速度*/
//				if(KEY_Z==1)//如果按下按键
//				{
//					yaw_target_INS_speed=0.05f;
//					calculate_yaw_motor_current_with_target_key_INS_speed();
//				}
//				if(KEY_X==1)//如果按下按键
//				{
//					yaw_target_INS_speed=-0.05f;
//					calculate_yaw_motor_current_with_target_key_INS_speed();
//				}					
//				if(	(KEY_Z_last==1	&&	KEY_Z==0)	||	(KEY_X_last==1	&&	KEY_X==0)	)yaw_target_INS_angle=yaw_INS_angle;
				
				if(KEY_Z==0 && KEY_X==0)//遥控器的优先级最低，其他都是零的时候才轮到它
				{
					if(yaw_target_INS_angle-yaw_INS_angle>180)yaw_INS_angle+=360;
					if(yaw_target_INS_angle-yaw_INS_angle<-180)yaw_INS_angle-=360;//让头走劣弧
					calculate_yaw_motor_current_with_target_INS_angle();
				}

			}

			
			yaw_moving_last_flag=yaw_moving_flag;
			KEY_Z_last=KEY_Z;
			KEY_X_last=KEY_X;
}

/**	[1.3]							
  * brief         云台跟随底盘模式下控制yaw轴
	* postscript										*/
void  Yaw_Motor_Control_in_Gimbal_Follow_Chassis_Mode()
{
		yaw_mode_flag=Yaw_Gimbal_Follow_Chassis_Mode;//标记一下yaw的模式
	
		yaw_encoder_speed=yaw_motor_data.speed_rpm;//读取yaw轴电机编码器速度
		yaw_encoder_angle=yaw_motor_data.ecd;//读取yaw轴电机编码器角度
	
		yaw_target_encoder_angle=Gimbal_Follow_Chassis_Offset_Ecd;//目标值为编码器偏置角度
	
		if(yaw_target_encoder_angle-yaw_encoder_angle>4096)yaw_encoder_angle+=8192;
		if(yaw_target_encoder_angle-yaw_encoder_angle<-4096)yaw_encoder_angle-=8192;//让头走劣弧
	
		calculate_yaw_motor_current_with_target_encoder_angle();//电机编码器的位置环
}


/**[2.1]					
  * brief					遥控模式下的陀螺仪速度环											
	* postscript						*/
void calculate_yaw_motor_current_with_target_INS_speed()
{
		yaw_motor_data.target_current	=	PID_calc(	&yaw_motor_INS_speed_pid,	yaw_INS_speed	,yaw_target_INS_speed);//yaw轴电机ID是1	
}

/**[2.2]					
  * brief					遥控模式下的陀螺仪位置环												
	* postscript							*/
void calculate_yaw_motor_current_with_target_INS_angle()
{
		yaw_target_INS_speed=PID_calc(	&yaw_motor_INS_angle_pid,	yaw_INS_angle,	yaw_target_INS_angle);

		yaw_motor_data.target_current=PID_calc(	&yaw_motor_INS_speed_pid, yaw_INS_speed, yaw_target_INS_speed);//yaw轴电机ID是1
}

/**[2.3]					
  * brief					自瞄模式下的陀螺仪速度环											
	* postscript					*/
void calculate_yaw_motor_current_with_autoaim_target_INS_speed()
{
		yaw_motor_data.target_current	=	PID_calc(	&yaw_motor_autoaim_INS_speed_pid,	yaw_INS_speed	,yaw_autoaim_target_INS_speed);//yaw轴电机ID是1	
}

/**[2.4]					
  * brief					自瞄模式下的陀螺仪位置环											
	* postscript							*/
void calculate_yaw_motor_current_with_autoaim_target_INS_angle()
{
		yaw_autoaim_target_INS_speed=PID_calc(	&yaw_motor_autoaim_INS_angle_pid,	yaw_INS_angle,	yaw_autoaim_target_INS_angle);

		yaw_motor_data.target_current=PID_calc(	&yaw_motor_autoaim_INS_speed_pid, yaw_INS_speed, yaw_autoaim_target_INS_speed);//yaw轴电机ID是1
}

/**[2.5]					
  * brief					云台跟随底盘模式下的编码器速度环											
	* postscript							*/
void calculate_yaw_motor_current_with_target_encoder_speed()
{
		yaw_motor_data.target_current=PID_calc(	&yaw_motor_encoder_speed_pid, yaw_encoder_speed, yaw_target_encoder_speed);//yaw轴电机ID是1
}

/**[2.6]					
  * brief					云台跟随底盘模式下的编码器位置环										
	* postscript							*/
void calculate_yaw_motor_current_with_target_encoder_angle()
{
		yaw_target_encoder_speed=PID_calc(	&yaw_motor_encoder_angle_pid,	yaw_encoder_angle,	yaw_target_encoder_angle);

		yaw_motor_data.target_current=PID_calc(	&yaw_motor_encoder_speed_pid, yaw_encoder_speed, yaw_target_encoder_speed);//yaw轴电机ID是1
}

/**[2.7]					
  * brief					按键微调模式下的陀螺仪速度环											
	* postscript		pid要单独调一调，不然直接用鼠标的pid会超调			*/
void calculate_yaw_motor_current_with_target_key_INS_speed()
{
		yaw_motor_data.target_current	=	PID_calc(	&yaw_motor_key_INS_speed_pid,	yaw_INS_speed	,yaw_target_INS_speed);//yaw轴电机ID是1	
}

/**	[3]							yaw轴电机初始化
  * brief         	初始化电机的数据信息和控制信息
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





/*********************************************************测试程序*********************************************************/
	//测试程序1：自瞄速度环测试程序
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

	//测试程序2：自瞄位置环测试程序
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
	if(yaw_autoaim_target_INS_angle-yaw_INS_angle<-180)yaw_INS_angle-=360;//让头走劣弧
	calculate_yaw_motor_current_with_autoaim_target_INS_angle();
*/

		//测试程序3：上自瞄测试程序
/*
	yaw_INS_angle=INS_angle_deg[0];
	yaw_INS_speed =	bmi088_real_data.gyro[2];
	
	if(AutoAim_Data_Receive.If_Aimed==1)//如果识别到装甲板
		yaw_autoaim_target_INS_angle=AutoAim_Data_Receive.Yaw;
	else//没识别到装甲板就定在原地
		yaw_autoaim_target_INS_angle=yaw_INS_angle;
	
	if(yaw_autoaim_target_INS_angle-yaw_INS_angle>180)yaw_INS_angle+=360;
	if(yaw_autoaim_target_INS_angle-yaw_INS_angle<-180)yaw_INS_angle-=360;//让头走劣弧
	calculate_yaw_motor_current_with_autoaim_target_INS_angle();
*/

		//测试程序4：一直进自瞄测试程序
/*
	yaw_INS_angle=INS_angle_deg[0];
	yaw_INS_speed =	bmi088_real_data.gyro[2];
	
	if(AutoAim_Data_Receive.If_Aimed==1)//如果识别到装甲板
	{	
		yaw_autoaim_target_INS_angle=AutoAim_Data_Receive.Yaw;
	
		if(yaw_autoaim_target_INS_angle-yaw_INS_angle>180)yaw_INS_angle+=360;
		if(yaw_autoaim_target_INS_angle-yaw_INS_angle<-180)yaw_INS_angle-=360;//让头走劣弧
		
		calculate_yaw_motor_current_with_autoaim_target_INS_angle();
	}
		else
			yaw_motor_data.target_current=0;
*/

		//测试程序5：yaw轴无力配合pitch调自瞄pid
/*
			yaw_motor_data.target_current=0;	
*/

		//测试程序6：云台跟随底盘速度环测试程序
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
