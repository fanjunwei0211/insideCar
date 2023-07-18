/**
  ****************************(C) COPYRIGHT 2023 DuanZheng****************************
  * @file       pitch_motor.c/h
  * @brief      河南因克斯智能科技有限公司制造，祝老板倾情打造
	*							EC-A4310-P2-36电机，我简称4310电机，两大特点
  *             一是英雄云台用的电机为10减速比，3nm扭矩，小体积大扭矩
  *             二是伺服控制模式号称加入磁链观测控制，好处就是PID不用调了
  * @note       速度环的Kp称为扰动补偿系数，Ki称为磁链观测增益？？？还是说反过来的，注释和说明书里说的不一样好像，
  *             位置环的Kp称为反馈补偿系数，Kd称为阻尼系数
	*							此外还加了温度保护、过载保护等一堆保护
  *        			靠通信设置参数不靠谱，得用上位机设置
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

#define MOTOR_ID 1						//电机ID
#define MOTOR4310_CAN hcan2		//电机通信can口
motor4310_data_t motor4310_data[8]; //后面那个接收电机数据的函数要用

/*********************************************************数据信息和控制信息*********************************************************/
motor_data_t 		pitch_motor_data;						//存放电机数据信息
pid_type_def		pitch_motor_INS_angle_pid;	//存放电机控制信息
pid_type_def		pitch_motor_INS_speed_pid;
pid_type_def		pitch_motor_mouse_INS_angle_pid;	
pid_type_def		pitch_motor_mouse_INS_speed_pid;	
pid_type_def		pitch_motor_autoaim_INS_angle_pid;	
pid_type_def		pitch_motor_autoaim_INS_speed_pid;	
/*********************************************************电机控制PID参数*********************************************************/
		/*****************遥控模式*****************/
		/*pitch轴			遥控模式			陀螺仪速度环*/
		fp32 pitch_motor_INS_speed_pid_Kp				=1700.0f;
		fp32 pitch_motor_INS_speed_pid_Ki				=60.0f;
		fp32 pitch_motor_INS_speed_pid_Kd				=0.0f;
		fp32 pitch_motor_INS_speed_pid_max_out	=1000.0f;
		fp32 pitch_motor_INS_speed_pid_max_iout	=600.0f;
		fp32 pitch_motor_INS_speed_PID[3];

		/*pitch轴			遥控模式			陀螺仪位置环*/
		fp32 pitch_motor_INS_angle_pid_Kp				=0.13f;//0.2f;
		fp32 pitch_motor_INS_angle_pid_Ki				=0.001f;//0.0f;
		fp32 pitch_motor_INS_angle_pid_Kd				=0.03f;//0.0f;
		fp32 pitch_motor_INS_angle_pid_max_out	=10.0f;//10.0f;
		fp32 pitch_motor_INS_angle_pid_max_iout	=2.0f;//0.0f;
		fp32 pitch_motor_INS_angle_PID[3];

		/*****************自瞄模式*****************/
		/*pitch轴			自瞄模式			陀螺仪速度环*/
		fp32 pitch_motor_autoaim_INS_speed_pid_Kp				=1700.0f;
		fp32 pitch_motor_autoaim_INS_speed_pid_Ki				=60.0f;
		fp32 pitch_motor_autoaim_INS_speed_pid_Kd				=0.0f;
		fp32 pitch_motor_autoaim_INS_speed_pid_max_out	=1000.0f;
		fp32 pitch_motor_autoaim_INS_speed_pid_max_iout	=600.0f;
		fp32 pitch_motor_autoaim_INS_speed_PID[3];
		
		/*pitch轴			自瞄模式			陀螺仪位置环*/
		fp32 pitch_motor_autoaim_INS_angle_pid_Kp				=0.20f;
		fp32 pitch_motor_autoaim_INS_angle_pid_Ki				=0.0f;
		fp32 pitch_motor_autoaim_INS_angle_pid_Kd				=0.0f;
		fp32 pitch_motor_autoaim_INS_angle_pid_max_out	=10.0f;
		fp32 pitch_motor_autoaim_INS_angle_pid_max_iout	=0.0f;
		fp32 pitch_motor_autoaim_INS_angle_PID[3];

		/*****************鼠标模式*****************/
		/*pitch轴			鼠标模式			陀螺仪速度环*/
		fp32 pitch_motor_mouse_INS_speed_pid_Kp				=1700.0f;
		fp32 pitch_motor_mouse_INS_speed_pid_Ki				=60.0f;
		fp32 pitch_motor_mouse_INS_speed_pid_Kd				=0.0f;
		fp32 pitch_motor_mouse_INS_speed_pid_max_out	=1000.0f;
		fp32 pitch_motor_mouse_INS_speed_pid_max_iout	=600.0f;
		fp32 pitch_motor_mouse_INS_speed_PID[3];

		/*pitch轴			鼠标模式			陀螺仪位置环*/
		fp32 pitch_motor_mouse_INS_angle_pid_Kp				=0.20f;
		fp32 pitch_motor_mouse_INS_angle_pid_Ki				=0.00f;
		fp32 pitch_motor_mouse_INS_angle_pid_Kd				=0.00f;
		fp32 pitch_motor_mouse_INS_angle_pid_max_out	=10.0f;
		fp32 pitch_motor_mouse_INS_angle_pid_max_iout	=0.0f;
		fp32 pitch_motor_mouse_INS_angle_PID[3];
/*********************************************************PITCH控制程序*********************************************************/
/*****pitch限位变量*****/
#define PITCH_INS_MAX 2.5f		//电机限位最大编码器角度7,-35
#define PITCH_INS_MIN -30.0f	//电机限位最小编码器角度
#define INS_BUFFER 2.0f				//电机限位角度附近的缓冲区

/*****pitch灵敏度变量*****/
//#define PITCH_RC_SENS 50.0f		//遥控器速度灵敏度水平	
float PITCH_RC_SENS=3.0f;		//遥控器速度灵敏度水平	
//#define PITCH_MOUSE_SENS 0.1f	//鼠标速度灵敏度
float PITCH_MOUSE_SENS=0.035f;	//鼠标速度灵敏度
float PITCH_MOUSE_AIM_ON_SENS=0.015f;//开镜之后鼠标灵敏度

/*****陀螺仪数据变量*****/
extern fp32 INS_angle_deg[3];//欧拉角(degree)

/*****pitch轴陀螺仪变量*****/
float pitch_INS_angle;									//pitch轴陀螺仪角度
float pitch_INS_speed;									//pitch轴陀螺仪角度

float pitch_target_motor_speed;					//pitch轴电机目标速度
float pitch_target_INS_speed;						//pitch轴陀螺仪目标速度
float pitch_target_INS_angle;						//pitch轴陀螺仪目标角度
float pitch_target_INS_speed_last;

float pitch_autoaim_target_INS_angle;		//pitch轴陀螺仪目标角度
float pitch_autoaim_target_INS_speed;		//pitch轴陀螺仪目标速度

/*****电机控制参数*****/
int16_t pitch_target_motor_current_or_torque;		//电流模式/力矩模式下的期望
uint8_t ack_status_value=2;			//报文类型

/*****pitch轴的控制模式*****/
#define Pitch_RC_Mode				0								//遥控模式
#define Pitch_Autoaim_Mode	1								//自瞄模式

/*****遥控器相关变量*****/
extern RC_ctrl_t rc_ctrl;
int16_t rc_ch1;//把遥控器的值赋给这个变量，目的是改上下俯仰逻辑的时候好改，比如杆往上推，只改一个正负号就能改它俯仰
int16_t rc_ch1_last;
int16_t rc_mouse_y;
int16_t rc_mouse_y_last;

/*****电机限位开启标志*****/
uint8_t max_limit_flag=1,min_limit_flag=1;//电机限位标志，标志为1是才有限位功能，不能一直限着，还得让电机回去

/*****按键微调*****/
int16_t KEY_C;
int16_t KEY_C_last;
int16_t KEY_B;
int16_t KEY_B_last;

/*****自瞄*****/
extern AutoAim_Data_Rx AutoAim_Data_Receive;

/*****pitch三个模式的标志变量*****/
uint8_t pitch_mode_flag;//0表示遥控;1表示自瞄
uint8_t pitch_mode_flag_last;

float pitch_target_INS_speed_last;
extern uint8_t aim_on_off_flag;//开镜标志位

void Pitch_Motor_Control(void)
{
	if(	(rc_ctrl.mouse.press_r)	&&	(AutoAim_Data_Receive.If_Aimed==1)	)//如果右键开启自瞄并且自瞄检测到目标
//	if(	((rc_ctrl.mouse.press_r)||rc_ctrl.rc.s[1]==1)	&&	(AutoAim_Data_Receive.If_Aimed==1)	)//只用遥控器测自瞄的时候，如果右键开启自瞄或者左拨杆拨到最上面  并且自瞄检测到目标（连上键鼠右键开启自瞄）
		Pitch_Motor_Control_in_Autoaim_Mode();
	else
	{
		Pitch_Motor_Control_in_RC_Mode();
	}
		
	
	pitch_mode_flag_last=pitch_mode_flag;

}


/**	[1.1]							
  * brief         遥控模式下控制pitch轴
	* postscript										*/
uint8_t using_mouse_flag;
uint8_t using_rc_flag;
uint8_t using_mouse_flag_last;
uint8_t using_rc_flag_last;
uint8_t pitch_moving_flag;
uint8_t pitch_moving_flag_last;
void  Pitch_Motor_Control_in_RC_Mode()
{
			/*(0)	读取pitch轴信息，进行模式过渡*/
		if(rc_ctrl.rc.s[1]==1||rc_ctrl.rc.s[1]==3)
		{
			pitch_mode_flag=Pitch_RC_Mode;//标记一下pitch的模式
					
			rc_ch1=-rc_ctrl.rc.ch[1];
			rc_mouse_y=-rc_ctrl.mouse.y;
			KEY_C=((rc_ctrl.key.v & KEY_PRESSED_OFFSET_C) >> 13);
			KEY_B=((rc_ctrl.key.v & KEY_PRESSED_OFFSET_B) >> 15);//读取键鼠信息
		
			pitch_INS_angle=INS_angle_deg[2];
			pitch_INS_speed =	bmi088_real_data.gyro[1];//pitch轴的陀螺仪速度（头向上抬速度为正）
			
		
			if(pitch_mode_flag==Pitch_RC_Mode && pitch_mode_flag_last!=Pitch_RC_Mode)		//如果刚切到遥控器模式,目标角度先定在当前角度
				pitch_target_INS_angle=pitch_INS_angle;																		
			
			
			/*(1)	电机正常工作角度范围*/
	
			if(pitch_INS_angle < PITCH_INS_MAX-INS_BUFFER  && 	pitch_INS_angle > PITCH_INS_MIN+INS_BUFFER)//如果电机在限制范围内
			{ 
				
//				/*(1.1)	动就速度控制*/
//				if(rc_ch1 != 0	||	rc_mouse_y!=0)//遥控器控制电机速度	
//				{
//					pitch_target_INS_speed=PITCH_RC_SENS*	rc_ch1/660.0f+PITCH_MOUSE_SENS*rc_mouse_y;
////					pitch_target_INS_speed=0.8f*pitch_target_INS_speed_last+0.2f*pitch_target_INS_speed;
////					pitch_target_INS_speed_last=pitch_target_INS_speed;//加低通滤波
//					
//					if(rc_ch1 != 0 && rc_mouse_y==0)
//						calculate_pitch_motor_current_or_torque_with_target_INS_speed();//如果遥控器在动并且鼠标没动，用遥控器的pid控制
//					else
//						calculate_pitch_motor_current_or_torque_with_target_mouse_INS_speed();//只要鼠标在动,就用鼠标的pid控制
//					
//					set_motor4310_cur_tor(MOTOR_ID,pitch_target_motor_current_or_torque,0,ack_status_value);//0表示电流控制	
//				}
//				
//				/*(1.2)	动变到不动，确定陀螺仪目标角度*/
//				if( (rc_ch1_last!=0 && rc_ch1 == 0) || (rc_mouse_y_last!=0 && rc_mouse_y!=0))//判断条件显然是错误的，但是利用程序执行的间隙，的
//					pitch_target_INS_angle=pitch_INS_angle;	
//				
//				/*(1.3)	不动就稳定在当前角度*/
//				if((rc_ch1==0) && (rc_mouse_y==0))
//				{	
//					if(KEY_C_last==0	&&	KEY_C!=0)pitch_target_INS_angle+=0.1f;
//					if(KEY_B_last==0	&&	KEY_B!=0)pitch_target_INS_angle-=0.1f;//按键微调目标值
//					
//					if(rc_ch1_last!=0 && rc_ch1 == 0)
//						calculate_pitch_motor_current_or_torque_with_target_INS_angle();//陀螺仪位置环
//					
//					if(rc_mouse_y_last!=0 && rc_mouse_y!=0)//判断条件显然是错误的
//						calculate_pitch_motor_current_or_torque_with_target_mouse_INS_angle();//陀螺仪位置环
//					
//					set_motor4310_cur_tor(MOTOR_ID,pitch_target_motor_current_or_torque,0,ack_status_value);//0表示电流控制
//				}


			if(aim_on_off_flag==0)//如果没开镜
				pitch_target_INS_speed=PITCH_RC_SENS*	rc_ch1/660.0f+PITCH_MOUSE_SENS*rc_mouse_y;
			else//如果开镜了
				pitch_target_INS_speed=PITCH_RC_SENS*	rc_ch1/660.0f+PITCH_MOUSE_AIM_ON_SENS*rc_mouse_y;
			 
			 pitch_target_INS_speed=0.8f*pitch_target_INS_speed_last+0.2f*pitch_target_INS_speed;
			 pitch_target_INS_speed_last=pitch_target_INS_speed;//加了个低通滤波
			 
			 if(rc_mouse_y!=0)
				 using_mouse_flag=1;
			 if(rc_ch1!=0)
				 using_rc_flag=1;
			 if(rc_mouse_y==0)
				 using_mouse_flag=0;
			 if(rc_ch1==0)
				 using_rc_flag=0;
			 
			if(rc_ch1!=0 ||rc_mouse_y!=0)	pitch_moving_flag=1;
			if(rc_ch1==0 &&	rc_mouse_y==0 && fabs(pitch_INS_speed)<0.5)	pitch_moving_flag=0;//根据遥控器摇杆和yaw的速度 两个条件同时判断；加上yaw速度判断可以防止yaw超调

			 
				if(using_mouse_flag==1)//如果在用鼠标
				{
					calculate_pitch_motor_current_or_torque_with_target_mouse_INS_speed();//只要鼠标在动,就用鼠标的pid控制
					set_motor4310_cur_tor(MOTOR_ID,pitch_target_motor_current_or_torque,0,ack_status_value);//0表示电流控制	
				}
				
				if((pitch_moving_flag_last==1)&&(pitch_moving_flag==0))
					pitch_target_INS_angle=pitch_INS_angle;	
				
				if(using_mouse_flag==0)
				{
					/*方案1：按键调整目标位置*/
//					if(KEY_C_last==0	&&	KEY_C!=0)pitch_target_INS_angle+=0.1f;
//					if(KEY_B_last==0	&&	KEY_B!=0)pitch_target_INS_angle-=0.1f;//按键微调目标值
					
					/*方案2：按键设置目标速度*/
					if(KEY_C==1)//如果按下按键
					{
						pitch_target_INS_speed=0.01f;
						calculate_pitch_motor_current_or_torque_with_target_mouse_INS_speed();//只要鼠标在动,就用鼠标的pid控制
						set_motor4310_cur_tor(MOTOR_ID,pitch_target_motor_current_or_torque,0,ack_status_value);//0表示电流控制	
					}
					if(KEY_B==1)//如果按下按键
					{
						pitch_target_INS_speed=-0.01f;
						calculate_pitch_motor_current_or_torque_with_target_mouse_INS_speed();//只要鼠标在动,就用鼠标的pid控制
						set_motor4310_cur_tor(MOTOR_ID,pitch_target_motor_current_or_torque,0,ack_status_value);//0表示电流控制	
					}					
					if(	(KEY_C_last==1	&&	KEY_C==0)	||	(KEY_B_last==1	&&	KEY_B==0)	)pitch_target_INS_angle=pitch_INS_angle;
										
					
					
					
					if(KEY_C==0 && KEY_B==0)//遥控器的优先级最低，其他都是零的时候才轮到它
					{
						if(using_rc_flag==0)
						{
							calculate_pitch_motor_current_or_torque_with_target_mouse_INS_angle();
							set_motor4310_cur_tor(MOTOR_ID,pitch_target_motor_current_or_torque,0,ack_status_value);//0表示电流控制	
						}
						if(using_rc_flag==1)
						{
							calculate_pitch_motor_current_or_torque_with_target_INS_speed();//如果遥控器在动并且鼠标没动，用遥控器的pid控制
							set_motor4310_cur_tor(MOTOR_ID,pitch_target_motor_current_or_torque,0,ack_status_value);//0表示电流控制	
						}
						if(using_rc_flag_last!=0 && using_rc_flag==0)
							pitch_target_INS_angle=pitch_INS_angle;	
					}

				}

			
			 using_mouse_flag_last=using_mouse_flag;
			 using_rc_flag_last=using_rc_flag;
				pitch_moving_flag_last=pitch_moving_flag;
			
			
				
				/*(1.4)	正常工作角度范围内，开启上下限位功能*/		
				max_limit_flag=min_limit_flag=1;
			}
			
			
						/*(2)	电机最大限位角度附近*/
			
			if(pitch_INS_angle >= PITCH_INS_MAX-INS_BUFFER)//如果电机进入限位缓冲区
			{
				/*(2.1)	如果限位功能开启，电机限位，然后关掉限位功能*/
				if(max_limit_flag==1)
				{
					pitch_target_INS_angle=PITCH_INS_MAX;
					calculate_pitch_motor_current_or_torque_with_target_INS_angle();//陀螺仪位置环
					set_motor4310_cur_tor(MOTOR_ID,pitch_target_motor_current_or_torque,0,ack_status_value);//0表示电流控制

					max_limit_flag=0;
				}
				
				/*(2.2)	如果还想控制电机往限位方向走，再开启限位功能，继续限位*/		
				if(rc_ch1<0 || rc_mouse_y<0) 
					max_limit_flag=1; 		
				
				/*(2.3)	如果控制电机离开限位附近，正常速度控制往回走*/		
//				if(rc_ch1>=0 || rc_mouse_y>=0)
				else
				{
					pitch_target_INS_speed=PITCH_RC_SENS*	rc_ch1/660.0f+PITCH_MOUSE_SENS*rc_mouse_y;
					calculate_pitch_motor_current_or_torque_with_target_INS_speed();//陀螺仪速度环
					set_motor4310_cur_tor(MOTOR_ID,pitch_target_motor_current_or_torque,0,ack_status_value);//0表示电流控制	
				}
			}
			
			
			/*(3)	电机最小限位角度附近*/		
			
			if(pitch_INS_angle<=PITCH_INS_MIN+INS_BUFFER)//如果电机进入限位缓冲区
			{
				/*(3.1)	如果限位功能开启，电机限位，然后关掉限位功能*/
				if(min_limit_flag==1)
				{
					pitch_target_INS_angle=PITCH_INS_MIN;
					calculate_pitch_motor_current_or_torque_with_target_INS_angle();//陀螺仪位置环
					set_motor4310_cur_tor(MOTOR_ID,pitch_target_motor_current_or_torque,0,ack_status_value);//0表示电流控制
					
					min_limit_flag=0;
				}
				
				/*(3.2)	如果还想控制电机往限位方向走，再开启限位功能，继续限位*/	
				if(rc_ch1>0 || rc_mouse_y>0)	
					min_limit_flag=1;	
				
				/*(3.3)	如果控制电机离开限位附近，正常速度控制往回走*/		
//				if(rc_ch1<=0 || rc_mouse_y<=0)
				else
				{
					pitch_target_INS_speed=PITCH_RC_SENS*	rc_ch1/660.0f+PITCH_MOUSE_SENS*rc_mouse_y;
					calculate_pitch_motor_current_or_torque_with_target_INS_speed();//陀螺仪速度环
					set_motor4310_cur_tor(MOTOR_ID,pitch_target_motor_current_or_torque,0,ack_status_value);//0表示电流控制			
				}				
			}
			
			
			
			rc_ch1_last=rc_ch1;
			rc_mouse_y_last=rc_mouse_y;
			KEY_C_last=KEY_C;
			KEY_B_last=KEY_B;
		}
		else
		{
			set_motor4310_cur_tor(MOTOR_ID,0,0,ack_status_value);//0表示电流控制
		}
		
}

/**	[1.2]							
  * brief         遥控模式下控制pitch轴
	* postscript										*/
static float pitch_last;
void  Pitch_Motor_Control_in_Autoaim_Mode()
{
		pitch_mode_flag=1;		//标记一下pitch的模式
	
		pitch_INS_angle=INS_angle_deg[2];//pitch角度读取姿态解算出的角度数据
		pitch_INS_speed =	bmi088_real_data.gyro[1];//pitch速度读取陀螺仪直接测出来的角速度数据，陀螺仪测的是三轴角速度
	
		pitch_autoaim_target_INS_angle=-AutoAim_Data_Receive.Pitch;//视觉发过来的pitch是头相对水平面的夹角，C板放反了，得加个负号；暂时没加前馈
		pitch_autoaim_target_INS_angle=0.8f*pitch_last+pitch_autoaim_target_INS_angle*0.2f;
		pitch_last=pitch_autoaim_target_INS_angle;//加低通滤波
	
		if(pitch_autoaim_target_INS_angle>PITCH_INS_MAX)pitch_autoaim_target_INS_angle=PITCH_INS_MAX;
		if(pitch_autoaim_target_INS_angle<PITCH_INS_MIN)pitch_autoaim_target_INS_angle=PITCH_INS_MIN;//做个限位
		
		calculate_pitch_motor_current_or_torque_with_autoaim_target_INS_angle();
		set_motor4310_cur_tor(MOTOR_ID,pitch_target_motor_current_or_torque,0,ack_status_value);//0表示电流控制	

}

/**[2.1]						
  * brief						自瞄位置环							
	* postscript						*/
void calculate_pitch_motor_current_or_torque_with_autoaim_target_INS_angle()
{
		pitch_autoaim_target_INS_speed=-PID_calc(&pitch_motor_autoaim_INS_angle_pid,	pitch_INS_angle,pitch_autoaim_target_INS_angle);
		pitch_target_motor_current_or_torque=PID_calc(&pitch_motor_autoaim_INS_speed_pid,	pitch_INS_speed,pitch_autoaim_target_INS_speed);
}

/**[2.2]					
  * brief					遥控陀螺仪速度环											
	* postscript							*/
void calculate_pitch_motor_current_or_torque_with_target_INS_speed()
{
		pitch_target_motor_current_or_torque=PID_calc(&pitch_motor_INS_speed_pid,	pitch_INS_speed,pitch_target_INS_speed);
}

/**[2.3]					
  * brief					遥控陀螺仪位置环											
	* postscript							*/
void calculate_pitch_motor_current_or_torque_with_target_INS_angle()
{
		pitch_target_INS_speed=-PID_calc(&pitch_motor_INS_angle_pid,	pitch_INS_angle,pitch_target_INS_angle);
		pitch_target_motor_current_or_torque=PID_calc(&pitch_motor_INS_speed_pid,	pitch_INS_speed,pitch_target_INS_speed);
}

/**[2.4]					
  * brief					鼠标陀螺仪速度环											
	* postscript							*/
void calculate_pitch_motor_current_or_torque_with_target_mouse_INS_speed()
{
		pitch_target_motor_current_or_torque=PID_calc(&pitch_motor_mouse_INS_speed_pid,	pitch_INS_speed,pitch_target_INS_speed);
}
/**[2.5]					
  * brief					鼠标陀螺仪位置环											
	* postscript							*/
void calculate_pitch_motor_current_or_torque_with_target_mouse_INS_angle()
{
		pitch_target_INS_speed=-PID_calc(&pitch_motor_mouse_INS_angle_pid,	pitch_INS_angle,pitch_target_INS_angle);
		pitch_target_motor_current_or_torque=PID_calc(&pitch_motor_mouse_INS_speed_pid,	pitch_INS_speed,pitch_target_INS_speed);
}
/**	[3]							
  * brief         	pitch轴pid初始化
	* postscript			初始化电机的数据信息和控制信息							*/
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



/*********************************************************测试程序*********************************************************/
//	/*测试程序1：电流模式初步尝试*/
//	if(rc_ctrl.rc.s[0]==3)
//	set_motor4310_cur_tor(MOTOR_ID,current_or_torque_expected,0,ack_status_value);//0表示电流控制
//	if(rc_ctrl.rc.s[0]==2)
//	set_motor4310_cur_tor(MOTOR_ID,0,0,ack_status_value);//0表示电流控制
	
//		/*测试程序2：电机速度环调试*/
//	if(rc_ctrl.rc.s[0]==1)
//	{
//		pitch_INS_speed =	bmi088_real_data.gyro[1];//pitch轴的陀螺仪速度（头向上抬速度为正）
//		pitch_target_INS_speed=0.5f;
//		calculate_pitch_motor_current_or_torque_with_target_INS_speed();
//		set_motor4310_cur_tor(MOTOR_ID,pitch_target_motor_current_or_torque,0,ack_status_value);//0表示电流控制
//	}
//	if(rc_ctrl.rc.s[0]==3)
//	{
//		pitch_INS_speed =	bmi088_real_data.gyro[1];//pitch轴的陀螺仪速度（头向上抬速度为正）
//		pitch_target_INS_speed=0.0f;
//		calculate_pitch_motor_current_or_torque_with_target_INS_speed();
//		set_motor4310_cur_tor(MOTOR_ID,pitch_target_motor_current_or_torque,0,ack_status_value);//0表示电流控制
//	}
//	if(rc_ctrl.rc.s[0]==2)
//	{
//		pitch_INS_speed =	bmi088_real_data.gyro[1];//pitch轴的陀螺仪速度（头向上抬速度为正）
//		pitch_target_INS_speed=-0.5f;
//		calculate_pitch_motor_current_or_torque_with_target_INS_speed();
//		set_motor4310_cur_tor(MOTOR_ID,pitch_target_motor_current_or_torque,0,ack_status_value);//0表示电流控制
//		
////			set_motor4310_cur_tor(MOTOR_ID,0,0,ack_status_value);//0表示电流控制
//	}

	
//		/*测试程序3：电机位置环调试*/
//		if(rc_ctrl.rc.s[0]==3)
//		{
//			pitch_INS_angle=INS_angle_deg[2];
//			pitch_target_INS_angle=0;
//			pitch_INS_speed =	bmi088_real_data.gyro[1];
//			calculate_pitch_motor_current_or_torque_with_target_INS_angle();//这句话之前是不是少了一个“	pitch_INS_speed =	bmi088_real_data.gyro[1];”??
//			set_motor4310_cur_tor(MOTOR_ID,pitch_target_motor_current_or_torque,0,ack_status_value);//0表示电流控制
//		}
//		if(rc_ctrl.rc.s[0]==2)
//		set_motor4310_cur_tor(MOTOR_ID,0,0,ack_status_value);//0表示电流控制


//			/*测试程序4：自瞄位置环跳跃测试*/
//			pitch_INS_angle=INS_angle_deg[2];
//			pitch_INS_speed =	bmi088_real_data.gyro[1];//pitch轴的陀螺仪速度（头向上抬速度为正）
//			if(rc_ctrl.rc.s[0]==1)
//				pitch_autoaim_target_INS_angle=-30.0f;
//			if(rc_ctrl.rc.s[0]==3)
//				pitch_autoaim_target_INS_angle=5.0f;
//			if(rc_ctrl.rc.s[0]==2)
//				pitch_autoaim_target_INS_angle=0.0f;
//			calculate_pitch_motor_current_or_torque_with_autoaim_target_INS_angle();
//			set_motor4310_cur_tor(MOTOR_ID,pitch_target_motor_current_or_torque,0,ack_status_value);//0表示电流控制	


//				/*测试程序5：上自瞄*/
//				pitch_INS_angle=INS_angle_deg[2];
//				pitch_INS_speed =	bmi088_real_data.gyro[1];//pitch轴的陀螺仪速度（头向上抬速度为正）
//				if(AutoAim_Data_Receive.If_Aimed==1)
//				{
//					pitch_autoaim_target_INS_angle=-AutoAim_Data_Receive.Pitch;
//				/*做个限位*/
//				if(pitch_autoaim_target_INS_angle>PITCH_INS_MAX)pitch_autoaim_target_INS_angle=PITCH_INS_MAX;
//				if(pitch_autoaim_target_INS_angle<PITCH_INS_MIN)pitch_autoaim_target_INS_angle=PITCH_INS_MIN;
//				
//				calculate_pitch_motor_current_or_torque_with_autoaim_target_INS_angle();
//				set_motor4310_cur_tor(MOTOR_ID,pitch_target_motor_current_or_torque,0,ack_status_value);//0表示电流控制	
//				}
//				else
//					set_motor4310_cur_tor(MOTOR_ID,0,0,ack_status_value);//0表示电流控制	
				
//				/*测试程序6：pitch定到-23的位置，配合yaw轴调自瞄pid*/
//				pitch_INS_angle=INS_angle_deg[2];
//				pitch_INS_speed =	bmi088_real_data.gyro[1];
//				pitch_autoaim_target_INS_angle=-23.0f;
//				calculate_pitch_motor_current_or_torque_with_autoaim_target_INS_angle();
//				set_motor4310_cur_tor(MOTOR_ID,pitch_target_motor_current_or_torque,0,ack_status_value);//0表示电流控制	




/*********************************************************说明书上的协议和控制函数*********************************************************/
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


/*****************************************[1]	电机设置指令*****************************************************************/
		
/**	[1.1]				
  * brief       设置电机通信模式  	
  * param[in]		motor_id:	电机ID
  * param[in]		cmd:	0x01：自动报文模式；	0x02：问答通信模式      
  * postscript	电机设置指令的标识符都是0x7FF；设置成功与否都有单独的报文反馈，格式见手册“2.1 电机通信模式设置指令”			*/
void set_motor4310_comm_mode(uint16_t motor_id,uint8_t cmd)
{
		if(cmd==0) return;//如果cmd是0，设置失败，返回
	
	
		CAN_TxHeaderTypeDef  motor_comm_mode_setting_tx_message;//comm:communication
		uint8_t              motor_comm_mode_setting_can_send_data[4];
		uint32_t  send_mail_box;
		motor_comm_mode_setting_tx_message.StdId=0x7FF;//仲裁场：标准帧的11位ID，ID：identifier标识符   目标设备的ID号  000-7FF范围是2的11次方
		motor_comm_mode_setting_tx_message.RTR=CAN_RTR_DATA;//仲裁场：RTR：Remote Transmission Request，远程发送请求；RTR位区别数据帧和远程帧
		motor_comm_mode_setting_tx_message.IDE=CAN_ID_STD;//控制场：IDE:Identifier Extension，标识符扩展，0表示不用扩展，就是标准标识符，1表示扩展标识符
		motor_comm_mode_setting_tx_message.DLC=0x04;//控制场：     DLC：Data Length Code数据长度代码
		
		motor_comm_mode_setting_can_send_data[0]=motor_id>>8;
		motor_comm_mode_setting_can_send_data[1]=motor_id&0xff;
		motor_comm_mode_setting_can_send_data[2]=0x00;
		motor_comm_mode_setting_can_send_data[3]=cmd;
	
		HAL_CAN_AddTxMessage(&MOTOR4310_CAN, &motor_comm_mode_setting_tx_message, motor_comm_mode_setting_can_send_data, &send_mail_box);
}

/**	[1.2]			
  * brief        	重置电机ID为0x01  
  * postscript		电机设置指令的标识符都是0x7FF；设置成功与否都有单独的报文反馈	，格式见手册“2.3 电机 CAN 通信 ID 设置指令”		*/
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
	*	brief					设置电机通信ID
  * param[in]			motor_id:电机ID；
	*	param[in]			motor_id_new:电机新ID	      
  * postscript	  电机设置指令的标识符都是0x7FF；设置成功有单独的报文反馈，失败则无应答，格式见手册“2.4 电机 CAN 通信 ID 重置指令”			*/
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

/*****************************************[2] 电机查询指令*****************************************************************/

/**	[2.1]			
	* brief					查询电机通信模式
  * param[in]			motor_id:电机ID；	      
  * postscript		电机查询指令的标识符都是0x7FF；查询成功与否都有单独的报文反馈，格式见手册“3.1 查询电机通信模式”			*/
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
  * brief			    查询电机通信ID
  * postscript		电机查询指令的标识符都是0x7FF；查询成功与否都有单独的报文反馈，格式见手册“3.2 查询CAN 通信 ID”			*/
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


/*****************************************[3] 电机控制指令*****************************************************************/

/**********************************[3.1] 电机控制指令(自动报文通信模式)****************************************/
/**	[3.1]					自动报文通信模式
  * brief         电机控制指令
  * param[in]			motor4310_data_t motor4310_data[8]：储存八个电机数据的结构体数组
  * param[in]			uint8_t motor_quantity：  电机数量1~8    
  * postscript		此通信协议可一帧数据控制最多 4 个电机，但是仅能控制电机的电流	；电机接收后会自动返回报文，格式见手册“4.1 自动报文通信模式控制指令”		*/
void set_motors4310_current(motor4310_data_t motor4310_data[8],uint8_t motor_quantity)
{
	uint16_t i;

	CAN_TxHeaderTypeDef motor4310_current_setting_tx_message;
	uint8_t motor4310_current_setting_can_send_data[8];
	uint32_t send_mail_box;
	
	/**(0)	设置要发送的数据帧的部分参数**/
	motor4310_current_setting_tx_message.RTR = CAN_RTR_DATA;
	motor4310_current_setting_tx_message.IDE = CAN_ID_STD;
	motor4310_current_setting_tx_message.DLC = 8;
	
	
	
	/**(1)	发送前四个电机can控制指令**/
	
		/*(1.1)	去除电机数量为零的情况*/
	if(motor_quantity<1)return;//如果电机数量是0，没必要跑下去了，返回吧
		/*(1.2)	设置数据帧的标识符*/
	motor4310_current_setting_tx_message.StdId = 0x1FF;
		/*(1.3)	设置数据帧的电流数据*/
	for(i=0;i<4;i++)
	{
		motor4310_current_setting_can_send_data[2*i]  = motor4310_data[i].current_desired_int>>8;//四个电机电流给定值高8位
		motor4310_current_setting_can_send_data[2*i+1]= motor4310_data[i].current_desired_int&0xff;//四个电机电流给定值低8位
	}
	i=0;//用完i，清零，后面接着用
		/*(1.4)	发送can数据帧*/
	while((HAL_CAN_AddTxMessage(&MOTOR4310_CAN,&motor4310_current_setting_tx_message,motor4310_current_setting_can_send_data,&send_mail_box)==HAL_ERROR)&&(i<0XFFF))
	{
		i++;	//等待发送结束
		if(i>=0XFFF)	break;
	}
	
	
	
	/**(2)	发送后四个电机can控制指令**/
	
		/*(2.1)	去除电机数量为零的情况*/
	if(motor_quantity<5)	return;//如果电机数量小于5，前面发四个电机已经够了，也没必要发后面四个了，返回吧
		/*(2.2)	设置数据帧的标识符*/
	motor4310_current_setting_tx_message.StdId = 0x2FF;
		/*(2.3)	设置数据帧的电流数据*/
	for(i=0;i<4;i++)
	{
		motor4310_current_setting_can_send_data[2*i]  = motor4310_data[i+4].current_desired_int>>8;//另外四个电机电流给定值高8位
		motor4310_current_setting_can_send_data[2*i+1]= motor4310_data[i+4].current_desired_int&0xff;//另外四个电机电流给定值低8位
	}
	i=0;
		/*(2.4)	发送can数据帧*/
	while((HAL_CAN_AddTxMessage(&MOTOR4310_CAN,&motor4310_current_setting_tx_message,motor4310_current_setting_can_send_data,&send_mail_box)==HAL_ERROR)&&(i<0XFFF))
	{
		i++;	//等待发送结束
		if(i>=0XFFF)	break;
	}
	
}

/**********************************[3.2] 电机控制指令(问答通信模式)****************************************/
/********************[3.2.1] 电机控制指令(问答通信模式)(不同控制模式)***********************/
/**	[3.2.1.1]			问答通信模式
  * brief         电机控制指令(问答通信模式)(力位混控模式)
  * param[in]			motor_id:1~0x7FE
  * param[in]			kp:0~4095 	对应 0.0f~500.0f。
  * param[in]			kd:0~511  	对应 0.0f~5.0f
  * param[in]			pos:0~65536 对应-12.5rad~12
  * param[in]			spd:0~4095 	对应-18.0rad/s~18.0rad/s
  * param[in]			tor:0~4095 	对应-30.0Nm~30.0Nm
  * postscript		问答通信模式下的电机控制指令能返回1~3三种报文类型，但是力位混控模式只返回报文类型1，无法选择返回其他类型，格式见手册“5 问答模式反馈报文”		*/
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
/* 在给定范围和位数的情况下，将浮点数转换为无符号整型数 */
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
	
	/*(1)	做个限幅*/
	if(kp>KP_MAX) 	kp=KP_MAX;	if(kp<KP_MIN) 	kp=KP_MIN;
	if(kd>KD_MAX) 	kd=KD_MAX;	if(kd<KD_MIN) 	kd=KD_MIN;	
	if(pos>POS_MAX)	pos=POS_MAX;if(pos<POS_MIN) pos=POS_MIN;
	if(spd>SPD_MAX)	spd=SPD_MAX;if(spd<SPD_MIN) spd=SPD_MIN;
	if(tor>T_MAX)		tor=T_MAX;	if(tor<T_MIN) 	tor=T_MIN;
		
	/*(2)	把float型数据转成uint型，因为can通信只能发uint型*/
  kp_int	= float_to_uint(	kp,		KP_MIN,		KP_MAX,		12	);//float型数据转换成unit12型数据
  kd_int	= float_to_uint(	kd,		KD_MIN,		KD_MAX, 	9		);
	pos_int = float_to_uint(	pos,	POS_MIN,	POS_MAX, 	16	);            
  spd_int = float_to_uint(	spd,	SPD_MIN, 	SPD_MAX, 	12	);
  tor_int = float_to_uint(	tor,	T_MIN, 		T_MAX, 		12	);	
	
	
	CAN_TxHeaderTypeDef motor_ctrl_cmd_tx_message;
	uint8_t motor_ctrl_cmd_can_send_data[8];
	uint32_t send_mail_box;
	
	/*(3)	设置数据帧发送参数*/
	motor_ctrl_cmd_tx_message.StdId = motor_id;//标识符为电机ID
	motor_ctrl_cmd_tx_message.IDE = CAN_ID_STD;
	motor_ctrl_cmd_tx_message.RTR = CAN_RTR_DATA;
	motor_ctrl_cmd_tx_message.DLC = 8;
	
	/*(4)	设置数据帧发送内容*/																																				
  motor_ctrl_cmd_can_send_data[0]=0x00|(kp_int>>7);											
	motor_ctrl_cmd_can_send_data[1]=((kp_int&0x7F)<<1)|((kd_int&0x100)>>8);
	motor_ctrl_cmd_can_send_data[2]=kd_int&0xFF;
	motor_ctrl_cmd_can_send_data[3]=pos_int>>8;
	motor_ctrl_cmd_can_send_data[4]=pos_int&0xFF;
	motor_ctrl_cmd_can_send_data[5]=spd_int>>4;
	motor_ctrl_cmd_can_send_data[6]=(spd_int&0x0F)<<4|(tor_int>>8);
	motor_ctrl_cmd_can_send_data[7]=tor_int&0xff;
	/*(4)	发送数据帧*/
  HAL_CAN_AddTxMessage(&MOTOR4310_CAN,&motor_ctrl_cmd_tx_message,motor_ctrl_cmd_can_send_data,&send_mail_box);
	
}

/**	[3.2.1.2]			问答通信模式
  * brief         电机控制指令(问答通信模式)(伺服位置控制模式)
  * param[in]			motor_id:1~0x7FE
  * param[in]			pos:输出端期望位置；数值即为实际角度，单位为弧度，例：120.5 表示120.5°
  * param[in]			spd:输出端期望速度；0~32767 对应 0~3276.7rpm，比例为 10
  * param[in]			cur:电机电流阈值；0~4095 对应 0~409.5A，比例为 10
	* param[in]			ack_status：报文返回状态；0：不返回  1：返回报文类型1  2：返回报文类型2  3：返回报文类型3
  * postscript		之前的注释写的：spd:0~18000	；cur:0~3000	；问答通信模式下的电机控制指令能返回1~3三种报文类型，格式见手册“5 问答模式反馈报文”	*/
void set_motor4310_position(uint16_t motor_id,float pos,uint16_t spd,uint16_t cur,uint8_t ack_status)
{
	CAN_TxHeaderTypeDef motor_pos_setting_tx_message;
	uint8_t motor_pos_setting_can_send_data[8];
	uint32_t send_mail_box;
	motor_pos_setting_tx_message.StdId = motor_id;
	motor_pos_setting_tx_message.RTR = CAN_RTR_DATA;
	motor_pos_setting_tx_message.IDE = CAN_ID_STD;
	motor_pos_setting_tx_message.DLC = 8;
	
	/*报文返回状态只有0到3*/
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

/**	[3.2.1.3]			问答通信模式
  * brief         电机控制指令(问答通信模式)(伺服速度控制模式)
  * param[in]			motor_id:1~0x7FE
  * param[in]			spd:输出端期望速度；CAN 数值与实际转速一一对应
  * param[in]			cur:电机电流阈值；0~65536 对应 0~6553.6A，比例为10
	* param[in]			ack_status：报文返回状态；0：不返回  1：返回报文类型1  2：返回报文类型2  3：返回报文类型3
  * postscript		跟伺服位置控制的数值解释还不一样，另外之前的注释写的：spd:-18000~18000	；cur:0~3000；问答通信模式下的电机控制指令能返回1~3三种报文类型，格式见手册“5 问答模式反馈报文”		*/
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

/**	[3.2.1.4]			问答通信模式
  * brief         电机控制指令(问答通信模式)(电流、力矩控制模式和刹车控制指令)
  * param[in]			motor_id:1~0x7FE
  * param[in]			cur_tor:     期望电流；-32768~32767 对应-327.68~327.67A   或是   期望力矩：-32768~32767 对应-327.68~327.67Nm
	* param[in]			ctrl_status：0：正常电流控制；1：力矩控制模式；2：变阻尼制动控制模式(电流参数自动忽略)；3：能耗制动控制模式(电流参数自动忽略)；4：再生制动控制模式(电流参数自动忽略)
	* param[in]			ack_status： 报文返回状态；0：不返回  1：返回报文类型1  2：返回报文类型2  3：返回报文类型3
  * postscript		问答通信模式下的电机控制指令能返回1~3三种报文类型，格式见手册“5 问答模式反馈报文”		*/
void set_motor4310_cur_tor(uint16_t motor_id,int16_t cur_tor,uint8_t ctrl_status,uint8_t ack_status)
{
	/*(1)	报文返回状态只有0到3，控制模式只有0到7，否则返回*/
	if(ack_status>3) 	return;
	if(ctrl_status>7)	return;
	
	/*(2)	如果ctrl_status！=0，就不是电流控制，限幅一下力矩*/
	if(ctrl_status) //进入扭矩控制模式或制动模式
	{
		if(cur_tor>3000) cur_tor=3000;
		else if(cur_tor<-3000) cur_tor=-3000;
	}
	
	/*(3)	如果ctrl_status==0,表示是电流控制，限幅一下电流*/
	else
	{
		if(cur_tor>2000) cur_tor=2000;
		else if(cur_tor<-2000) cur_tor=-2000;
	}
	
	/*(4)	can发送*/
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
/********************[3.2.2] 电机控制指令(问答通信模式)(参数配置)***********************/
/**	[3.2.2.1]			问答通信模式
  * brief         电机控制指令(问答通信模式)(加速度配置)
  * param[in]			motor_id:1~0x7FE
  * param[in]			acc:    加速度数值：0~2000 对应 0~20rad/s，内部已限幅。加速度减小，运行会更柔和，但是过小易超调，系统默认值为 2000
	* param[in]			ack_status： 报文返回状态；		0：不返回		1：返回报文类型4(就是4)	2：不返回，并且驱动不响应，设置失败	3：不返回，并且驱动不响应，设置失败
  * postscript		问答通信模式下的电机配置指令只能返回报文类型4，格式见手册“5 问答模式反馈报文”		*/
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

/**	[3.2.2.2]			问答通信模式
  * brief         电机控制指令(问答通信模式)(磁链观测增益和扰动补偿系数配置)
  * param[in]			motor_id:1~0x7FE
  * param[in]			linkage：速度环的Kp叫扰动补偿系数；0~10000 对应 vesc tool 中的 0~1。默认值 60，对应 0.006
  * param[in]			speedKI：速度环的KI叫磁链观测增益；0~10000 对应 vesc tool 中的 0~1。默认值 1000，对应 0.1。
	* param[in]			ack_status： 报文返回状态；		0：不返回		1：返回报文类型4(就是4)	2：不返回，并且驱动不响应，设置失败	3：不返回，并且驱动不响应，设置失败
  * postscript		问答通信模式下的电机配置指令只能返回报文类型4，格式见手册“5 问答模式反馈报文”		*/
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

/**	[3.2.2.3]			问答通信模式
  * brief         电机控制指令(问答通信模式)(电机反馈补偿系数与阻尼系数（KD）配置)
  * param[in]			motor_id:1~0x7FE
  * param[in]			fdbKP： 位置环的Kp称为反馈补偿系数，电机反馈补偿系数：0~10000 对应 vesc tool 中的 0~1。默认值60，对应0.006
  * param[in]			fdbKD：	位置环的Kd称为阻尼系数：0~10000 对应 vesc tool 中的 0~1。默认值 0
	* param[in]			ack_status： 报文返回状态；		0：不返回		1：返回报文类型4(就是4)	2：不返回，并且驱动不响应，设置失败	3：不返回，并且驱动不响应，设置失败
  * postscript		问答通信模式下的电机配置指令只能返回报文类型4，格式见手册“5 问答模式反馈报文”		*/
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

/********************[3.2.3] 电机控制指令(问答通信模式)(参数查询)***********************/
/**	[3.2.3.1]			问答通信模式
  * brief         电机控制指令(问答通信模式)(控制参数查询)
  * param[in]			motor_id:1~0x7FE
  * param[in]			param_cmd： 查询代码  0：保留，目前无效；1：查询当前位置；2：查询当前速度；3：查询当前电流；4：查询当前功率
																				5：查询当前加速度；6：查询当前磁链观测增益；7：查询当前扰动补偿系数；8：查询反馈补偿系数；9：查询阻尼系数
  * postscript		问答通信模式下的电机查询指令只能返回报文类型5，格式见手册“5 问答模式反馈报文”		*/
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

/*****************************************[4] 电机返回报文接收*****************************************************************/

motor4310_feedback_t motor4310_feedback;
uint16_t motor_id_check=0;

void get_motor4310_data(CAN_RxHeaderTypeDef *RxMessage,uint8_t RxData[],uint8_t comm_mode)
{
	uint8_t motor_id_t=0;
	uint8_t ack_status=0;
	int pos_int=0;            
	int spd_int=0;
	int cur_int=0;
	
	
	/*(1)		[1]电机设置指令和[2]电机查询指令都有单独格式的报文返回，返回报文的标识符都是0x7FF*/
	if(RxMessage->StdId==0x7FF)
	{	
		/*(1.1)	通过Byte3位来判断是不是电机返回的报文*/
		/*(1.1.1)	如果Byte3位不是0x01就直接返回*/
		if(RxData[2]!=0x01)//Byte2 位：0x01表示是电机返回给单片机的报文，0x00表示是单片机往电机发送的指令，参照“2.1电机通信模式设置指令”
			return;	
		
		/*(1.2)	通过Byte0和Byte1位来判断是不是“查询CAN通信ID”*/
		/*(1.2.1)如果Byte0和Byte1位都是0xFF，表示“查询CAN通信ID”查询成功*/
		if((RxData[0]==0xff)&&(RxData[1]==0xFF))
		{
			motor4310_feedback.motor_id=RxData[3]<<8|RxData[4];//第3个字节和第四个字节分别存储电机ID的高低8位
			motor4310_feedback.motor_fbd=0x01;
		}
		/*(1.2.2)如果Byte0和Byte1位都是0x80，表示“查询CAN通信ID”查询失败*/
		else if((RxData[0]==0x80)&&(RxData[1]==0x80))//inquire failed
		{
			motor4310_feedback.motor_id=0;
			motor4310_feedback.motor_fbd=0x80;
		}
		
		/*(1.3)	通过Byte0和Byte1位来判断是不是“电机CAN通信ID重置指令”*/
		/*(1.3.1)如果Byte0和Byte1位都是0x7F，表示“电机CAN通信ID重置指令”重置成功，失败的话无应答*/
		else if((RxData[0]==0x7F)&&(RxData[1]==0x7F))//reset ID succeed
		{
			motor4310_feedback.motor_id=1;
			motor4310_feedback.motor_fbd=0x05;
		}
		/*(1.4)	剩下的指令 返回报文的Byte0和Byte1位分别是电机ID的高低8位，可以提取出电机的ID信息*/
		else
		{
			motor4310_feedback.motor_id=RxData[0]<<8|RxData[1];
			motor4310_feedback.motor_fbd=RxData[3];
		}
	}
	
	/*(2)		如果是[3]电机控制指令(自动报文模式)返回的报文，在调用函数的时候把入口参数comm_mode设为0x01*/
		else if(comm_mode==0x01)//表示自动报文模式automatic feedback mode
	{
		motor_id_t=RxMessage->StdId-0x205;
		motor4310_data[motor_id_t].angle_actual_int=(uint16_t)(RxData[0]<<8|RxData[1]);//输出端位置
		motor4310_data[motor_id_t].speed_actual_int=(int16_t)(RxData[2]<<8|RxData[3]);//输出端转速
		motor4310_data[motor_id_t].current_actual_int=(RxData[4]<<8|RxData[5]);//我看说明书应该是实际转矩吧，不过转矩和电流应该也是关联的
		motor4310_data[motor_id_t].temperature=RxData[6];//电机温度
		motor4310_data[motor_id_t].error=RxData[7];//错误信息
	}
	
	/*(2)		如果是[3]电机控制指令(问答通信模式)返回的报文，在调用函数的时候把入口参数comm_mode设为0x00*/
	else if(comm_mode==0x00)//表示问答模式Response mode
	{
																				
		ack_status=RxData[0]>>5;			//第一个字节5到7位表示报文类型：1-3表示控制指令返回报文的三个类型，4表示配置指令返回报文，5表示查询指令返回报文
		motor_id_t=RxMessage->StdId-1;
		motor_id_check=RxMessage->StdId;
		motor4310_data[motor_id_t].motor_id=motor_id_t;
		motor4310_data[motor_id_t].error=RxData[0]&0x1F;//第一个字节0到4位表示电机错误信息
		
		/*(2.1)	如果是[3]电机控制指令(问答通信模式)(不同控制模式)控制指令，返回的报文类型1*/
		if(ack_status==1)
		{
			/*接收int型数据：位置、转速、电流*/
			pos_int=RxData[1]<<8|RxData[2];//输出端位置
			spd_int=RxData[3]<<4|(RxData[4]&0xF0)>>4;//输出端转速
			cur_int=(RxData[4]&0x0F)<<8|RxData[5];//电机相电流
			/*转换一下数据类型 赋给 存电机信息的结构体，顺便把温度信息也拿出来 */
			motor4310_data[motor_id_t].angle_actual_rad=uint_to_float(pos_int,POS_MIN,POS_MAX,16);
			motor4310_data[motor_id_t].speed_actual_rad=uint_to_float(spd_int,SPD_MIN,SPD_MAX,12);
			motor4310_data[motor_id_t].current_actual_float=uint_to_float(cur_int,I_MIN,I_MAX,12);
			motor4310_data[motor_id_t].temperature=(RxData[6]-50)/2;
		}
		
		/*(2.2)	如果是[3]电机控制指令(问答通信模式)(不同控制模式)控制指令，返回的报文类型2*/		
		else if(ack_status==2)
		{
			/*先赋给共用体方便后面数据类型转换*/
			rv_type_convert.buf[0]=RxData[4];
			rv_type_convert.buf[1]=RxData[3];
			rv_type_convert.buf[2]=RxData[2];
			rv_type_convert.buf[3]=RxData[1];
			
			motor4310_data[motor_id_t].angle_actual_float=rv_type_convert.to_float;//输出端位置
			motor4310_data[motor_id_t].current_actual_int=RxData[5]<<8|RxData[6];//电机相电流
			motor4310_data[motor_id_t].temperature=(RxData[7]-50)/2;//电机温度
			motor4310_data[motor_id_t].current_actual_float=motor4310_data[motor_id_t].current_actual_int/100.0f;
		}
		
		/*(2.3)	如果是[3]电机控制指令(问答通信模式)(不同控制模式)控制指令，返回的报文类型3*/		
		else if(ack_status==3)//如果是问答通信模式返回报文类型3，是控制指令返回的报文类型之一  response frame 3
		{
			rv_type_convert.buf[0]=RxData[4];
			rv_type_convert.buf[1]=RxData[3];
			rv_type_convert.buf[2]=RxData[2];
			rv_type_convert.buf[3]=RxData[1];
			
			motor4310_data[motor_id_t].speed_actual_float=rv_type_convert.to_float;//输出端速度
			motor4310_data[motor_id_t].current_actual_int=RxData[5]<<8|RxData[6];//电机相电流
			motor4310_data[motor_id_t].temperature=(RxData[7]-50)/2;//电机温度
			motor4310_data[motor_id_t].current_actual_float=motor4310_data[motor_id_t].current_actual_int/100.0f;
		}
		
		/*(2.4)	如果是[3]电机控制指令(问答通信模式)(参数配置)配置指令，返回的报文类型4*/
		else if(ack_status==4)//如果是问答通信模式返回报文类型4，是配置指令返回的报文类型 response frame 4
		{
			if(RxMessage->DLC!=3)	return;
			motor4310_feedback.INS_code=RxData[1];//配置代码
			motor4310_feedback.motor_fbd=RxData[2];//配置状态，0表示失败，1表示成功
		}
		
		/*(2.5)	如果是[3]电机控制指令(问答通信模式)(参数查询)查询指令，返回的报文类型5*/
		else if(ack_status==5)//如果是问答通信模式返回报文类型5，是查询指令返回的报文类型 response frame 5
		{
			/*(2.5.1)	读取一下查询代码，代码范围1-9*/
			motor4310_feedback.INS_code=RxData[1];
			
			/*(2.5.2) 根据查询代码代表的意义响应赋值*/
			/*(2.5.2.1)	1：位置数据*/
			if(motor4310_feedback.INS_code==1&RxMessage->DLC==6)
			{
				rv_type_convert.buf[0]=RxData[5];
				rv_type_convert.buf[1]=RxData[4];
				rv_type_convert.buf[2]=RxData[3];
				rv_type_convert.buf[3]=RxData[2];
				
				motor4310_data[motor_id_t].angle_actual_float=rv_type_convert.to_float;//当前位置（float型）
			}
			/*(2.5.2.2)	2：速度数据*/
			else if(motor4310_feedback.INS_code==2&RxMessage->DLC==6)//
			{
				rv_type_convert.buf[0]=RxData[5];
				rv_type_convert.buf[1]=RxData[4];
				rv_type_convert.buf[2]=RxData[3];
				rv_type_convert.buf[3]=RxData[2];
				
				motor4310_data[motor_id_t].speed_actual_float=rv_type_convert.to_float;//当前速度（float型）
			}
			/*(2.5.2.3)	3：电流数据*/
			else if(motor4310_feedback.INS_code==3&RxMessage->DLC==6)//
			{
				rv_type_convert.buf[0]=RxData[5];
				rv_type_convert.buf[1]=RxData[4];
				rv_type_convert.buf[2]=RxData[3];
				rv_type_convert.buf[3]=RxData[2];
				motor4310_data[motor_id_t].current_actual_float=rv_type_convert.to_float;//当前电流（float型）
			}
			/*(2.5.2.4)	4：功率数据*/
			else if(motor4310_feedback.INS_code==4&RxMessage->DLC==6)//get power
			{
				rv_type_convert.buf[0]=RxData[5];
				rv_type_convert.buf[1]=RxData[4];
				rv_type_convert.buf[2]=RxData[3];
				rv_type_convert.buf[3]=RxData[2];
				motor4310_data[motor_id_t].power=rv_type_convert.to_float;//当前功率（float型）
			}
			/*(2.5.2.5)	5：加速度数据*/
			else if(motor4310_feedback.INS_code==5&RxMessage->DLC==4)//get acceleration
			{
				motor4310_data[motor_id_t].acceleration=RxData[2]<<8|RxData[3];//当前加速度（uint16型）
			}
			/*(2.5.2.6)	6：磁链观测增益*/
			else if(motor4310_feedback.INS_code==6&RxMessage->DLC==4)//get linkage_KP
			{
				motor4310_data[motor_id_t].linkage_KP=RxData[2]<<8|RxData[3];//当前磁链观测增益（uint16型）
			}
			/*(2.5.2.7)	7：扰动补偿系数*/
			else if(motor4310_feedback.INS_code==7&RxMessage->DLC==4)//get speed_KI
			{
				motor4310_data[motor_id_t].speed_KI=RxData[2]<<8|RxData[3];//当前扰动补偿系数（uint16型）
			}
			/*(2.5.2.8)	8：反馈补偿增益*/
			else if(motor4310_feedback.INS_code==8&RxMessage->DLC==4)//get feedback_KP
			{
				motor4310_data[motor_id_t].feedback_KP=RxData[2]<<8|RxData[3];//当前反馈补偿增益（uint16型）
			}
			/*(2.5.2.9)	9：阻尼系数*/
			else if(motor4310_feedback.INS_code==9&RxMessage->DLC==4)//get feedback_KD
			{
				motor4310_data[motor_id_t].feedback_KD=RxData[2]<<8|RxData[3];//当前阻尼系数（uint16型）
			}

		}
	}
	
}




