#include "shoot_task.h"
#include "fric_motor.h"
#include "FreeRTOS.h"
#include "task.h"
#include "remote_control.h"
#include "dial_motor.h"
#include "motor_can.h"
#include "ist8310driver.h"
#include "main.h"
#include "BMI088driver.h"
#include "vofa.h"
#include "stdlib.h"//要调用绝对值函数，在摩擦轮掉速判断那边
#include "Referee.h"
#include "pitch_motor.h"
#include "aim_motor.h"
#include "Nmanifold_usart_task.h"


#define ABS(x) ( (x)>0?(x):-(x) )

/*摩擦轮can通信pid控制*/
//#define FRIC_SPEED_10 4800
//#define FRIC_SPEED_16 6000
float FRIC_SPEED_10=3000;
float FRIC_SPEED_16=4000;
/*摩擦轮pwm控制*/
#define fric_0m 1000//1500
//#define fric_10m 1400//1500
//#define fric_16m 1540//1510
uint16_t fric_10m=1475;//1400
uint16_t fric_14m=1670 ;//1530,1475稳定14.0±0.1，1480稳定14.4±0.2
uint16_t fric_15m=1500;//弹速还没校过，先拿这个用用
int16_t fric_test=1000;
uint16_t fric_target=1475;
/*****3508参数宏定义*****/
#define Delta_3508_Ecd_After_One_Round 8192			//3508转一圈之后编码器数值改变量
#define Reduction_Ratio_of_3508_Motor 3591/187		//3508电机的减速比(宏定义的数字参与运算，后面不加分号，不然乘法运算时会被当成指针)
#define Delta_Total_3508_Ecd_After_One_Round Delta_3508_Ecd_After_One_Round*Reduction_Ratio_of_3508_Motor//用成宏定义避免精度损失
/*****2006参数宏定义*****/
#define Delta_2006_Ecd_After_One_Round 8192			//3508转一圈之后编码器数值改变量
#define Reduction_Ratio_of_2006_Motor 36		//3508电机的减速比(宏定义的数字参与运算，后面不加分号，不然乘法运算时会被当成指针)
#define Delta_Total_2006_Ecd_After_One_Round Delta_2006_Ecd_After_One_Round*Reduction_Ratio_of_2006_Motor//用成宏定义避免精度损失

/*****extern电机信息*****/
extern motor_data_t 		dial_motor_data[2];
extern motor_data_t 		fric_motor_data[2];
extern motor4310_data_t motor4310_data[8];
extern motor_data_t aim_motor_data;
extern ext_game_robot_state_t Game_Robot_State;
ext_game_robot_state_t Game_Robot_State_last;
/*****上拨弹盘转固定角度相关变量*****/
int64_t up_dial_reset_offset_total_ecd=0;			 //上拨弹盘复位偏置总编码器角度；复位偏置总编码器角度，也就是从这个总编码器角度基础上转固定角度
uint16_t up_dial_rotate_cnt=0;//上拨弹盘旋转次数

/*****上下拨弹盘的堵转标志变量和堵转计数*****/
uint8_t up_dial_start_reset_locked_rotor_flag=0;//上拨弹盘	刚上电	复位方向	堵转标志变量
uint8_t up_dial_start_reset_locked_rotor_flag_last;
uint16_t up_dial_start_reset_locked_rotor_cnt=0;//上拨弹盘	刚上电	复位方向	堵转计数

uint8_t up_dial_reset_locked_rotor_flag=0;			//上拨弹盘					复位方向	堵转标志变量
uint8_t up_dial_reset_locked_rotor_flag_last=0;//上拨弹盘复位方向堵转标志变量
uint16_t up_dial_reset_locked_rotor_cnt=0;			//上拨弹盘					复位方向	堵转计数

uint8_t up_dial_dial_locked_rotor_flag=0;				//上拨弹盘					拨弹方向	堵转标志位
uint16_t up_dial_dial_locked_rotor_cnt=0;				//上拨弹盘					拨弹方向	堵转计数

uint8_t down_dial_locked_rotor_flag=0;					//下拨弹盘										堵转标志变量
uint8_t down_dial_locked_rotor_flag_last;	
uint16_t down_dial_locked_rotor_cnt=0;					//下拨弹盘										堵转计数

RC_ctrl_t rc_ctrl_last;
int16_t KEY_CTRL;
int16_t KEY_CTRL_last;
uint8_t fric_duandian_flag;
uint16_t fric_duandian_cnt;
uint8_t fric_shangdian_flag;
uint8_t fric_shangdian_flag_last;
uint16_t fric_shangdian_cnt;

uint8_t fric_target_init_flag;//裁判系统射速上限是16m/s的时候，因为按ctrl要切换速度，所以得先初始化一个弹速，因为只需要初始化一次，所以这个标志位初始化完就置一，也不改了
uint16_t fric_UI_flag;

extern AutoAim_Data_Rx AutoAim_Data_Receive;
AutoAim_Data_Rx AutoAim_Data_Receive_last;
uint16_t autoaim_shoot_allowed_number;
void shoot_task(void const * argument)
{
	fric_motor_init();//摩擦轮数据信息和控制信息初始化
	dial_motor_init();
	aim_motor_init();
	while(1)
	{
		Aim_Motor_Control();//瞄准镜控制
		
		/*(1)	开启摩擦轮*/
		/*pwm控制*/
		KEY_CTRL=((rc_ctrl.key.v & KEY_PRESSED_OFFSET_CTRL) >> 5);//读取按键
		
			/*默认弹速设置（根据裁判系统）*/
			if(Game_Robot_State.shooter_id1_42mm_speed_limit==16)		
			{
				if(fric_target_init_flag==0)
				{
					fric_target=fric_14m;
					fric_target_init_flag=1;
				}
				
				if(KEY_CTRL_last==0 && KEY_CTRL==1)//按crtl切换弹速
					fric_target=	(fric_target==fric_14m)?fric_15m:fric_14m;
			}
			else 
				fric_target=fric_10m;
			
			if(Game_Robot_State_last.mains_power_shooter_output==1 && Game_Robot_State.mains_power_shooter_output==0)//如果发射机构下电又上电，得先让他上电工作的pwm持续1000一段时间，不然会进电调保护
			{
				fric_duandian_flag=1;//进摩擦轮上电保护模式，一开始先持续一段时间pwm1000状态
			}
			if(Game_Robot_State_last.mains_power_shooter_output==0 && Game_Robot_State.mains_power_shooter_output==1)//如果发射机构下电又上电，得先让他上电工作的pwm持续1000一段时间，不然会进电调保护
			{
				fric_shangdian_flag=1;//进摩擦轮上电保护模式，一开始先持续一段时间pwm1000状态
			}
			
			if(fric_duandian_flag==1)
				fric_target=1000;
			
			if(fric_shangdian_flag==1)
			{
				fric_target=1000;
				fric_duandian_flag=0;
				fric_shangdian_cnt++;
				if(fric_shangdian_cnt>600)
				{
					fric_shangdian_cnt=0;
					fric_shangdian_flag=0;
				}
			}
			
			if(fric_shangdian_flag_last==1 &&	fric_shangdian_flag==0)//消除下电再上电摩擦轮转速不更新的bug
				fric_target_init_flag=0;
		
			if(fric_target!=1000 && Game_Robot_State.mains_power_shooter_output==1 && (rc_ctrl.rc.s[0]==1 || rc_ctrl.rc.s[0]==3)	)fric_UI_flag=1;
			else fric_UI_flag=0;
			
			
			fric_rotate_by_PWM(fric_target);
			KEY_CTRL_last=KEY_CTRL;
			fric_shangdian_flag_last=fric_shangdian_flag;
//		
//		TIM1->CCR1=fric_test;
//		TIM1->CCR2=fric_test;
		
		/*can控制*/
//		if(motor4310_data[0].angle_actual_float>50)//pitch抬不起来不能转摩擦轮
//		{
//			if(Game_Robot_State.shooter_id1_42mm_speed_limit==16)		
//				fric_rotate_at_certain_speed(FRIC_SPEED_16);
//			else 
//				fric_rotate_at_certain_speed(FRIC_SPEED_10);
//		}
//		else
//			fric_rotate_at_certain_speed(0);

		
		/*(2)	启动拨弹盘*/
		
		
		
		/*(2.1)	拨弹盘堵转检测*/
		locked_rotor_detect(dial_motor_data[0].give_current,700,70,&up_dial_dial_locked_rotor_flag,&up_dial_dial_locked_rotor_cnt);//上拨弹盘拨弹堵转检测

		
		/*解决检录和死亡时的bug*/
		if(Game_Robot_State.mains_power_chassis_output==0)//如果电管chassis接口断了
			up_dial_start_reset_locked_rotor_flag=0;//重新上电复位
		
		/*(2.2)	上拨弹盘上电复位*/
		if(up_dial_start_reset_locked_rotor_flag==0)
			up_dial_reset();
		
		/*(2.3)	上下拨弹盘配合打弹*/
		else
		{			

			if(rc_ctrl.mouse.press_r==0)//如果没按下鼠标右键，就正常打弹
			{	
					/*(2.3.1)	如果上拨弹盘能正常拨弹没有发生堵转*/
					if(up_dial_dial_locked_rotor_flag==0)
					{
						up_dial_rotate_certain_angle(120.0f);//上拨弹盘转固定角度
						down_dial_rotate_until_locked_rotor();//上拨弹盘转到目标位置之后，下拨弹盘转到堵转
					}
					
					
					/*(2.3.2)	如果上拨弹盘堵转*/
					if(up_dial_dial_locked_rotor_flag==1)
					{
						up_dial_reset_and_Reset_Direction_locked_rotor_detect();
						down_dial_rotate_until_locked_rotor();//上拨弹盘转到目标位置之后，下拨弹盘转到堵转		//这里要不要加这句话，因为如果没有转到目标位置下拨弹盘似乎也不会动			
					}
				
			}
			else//			如果按下鼠标右键，进入自瞄模式																											如果左拨杆拨到最上面，进自瞄模式
			{
				
				if(rc_ctrl_last.mouse.press_r==0 && rc_ctrl.mouse.press_r==1)//如果刚点鼠标右键进入自瞄模式，清空一下允许发弹量
						autoaim_shoot_allowed_number=0;
				
//				if(rc_ctrl_last.rc.s[0]!=1 && rc_ctrl.rc.s[0]==1)//只用遥控器测试自瞄的条件   如果检测到右拨杆从中间拨到上面的跳变沿
				if(rc_ctrl_last.mouse.press_l==0 && rc_ctrl.mouse.press_l==1)
						autoaim_shoot_allowed_number=1;//允许发弹量++
				
				if(AutoAim_Data_Receive_last.Shoot_Freq==0 && AutoAim_Data_Receive.Shoot_Freq>0)//如果视觉传回来能打弹的跳变沿
					if(autoaim_shoot_allowed_number>0)//如果允许发弹量大于零
					{
						up_dial_rotate_cnt++;//转动的总次数加一
						dial_motor_data[0].target_total_ecd=+up_dial_rotate_cnt*Delta_Total_2006_Ecd_After_One_Round*120.0f/360.0f+up_dial_reset_offset_total_ecd;//设置目标转动角度；加负号才能让拨弹盘正着转；用fp32数据类型存储最大是3.4*10的38次方，范围应该是够的，内存应该也是够的
					
						autoaim_shoot_allowed_number--;
						 
					}
				
					down_dial_rotate_until_locked_rotor();
					calculate_up_dial_motor_current_with_target_total_angle();
					
			}
			
		}
		
		rc_ctrl_last=rc_ctrl;
		Game_Robot_State_last=Game_Robot_State;
		AutoAim_Data_Receive_last=AutoAim_Data_Receive;
		
		vTaskDelay(2);
		send_motor_3508current_through_CAN2(fric_motor_data[0].target_current,fric_motor_data[1].target_current,aim_motor_data.target_current,0);//发两个摩擦轮的电流
		vTaskDelay(2);
		send_motor_3508_or_2006_current_through_CAN1(dial_motor_data[1].target_current,dial_motor_data[0].target_current,0,0);//发两个拨弹盘的电流
		vTaskDelay(2);
		
		/*vofa显示波形(别在其他文件里也发，不然busy)*/
		//send_data_to_vofa4(dial_motor_data[1].speed_rpm,dial_motor_data[1].target_speed_rpm,10,0);
	}
	
}


/**	[1.1]							
  * brief         	刚上电，上拨弹盘复位
	* postscript			上拨弹盘往复位方向转，并进行堵转检测							*/
void up_dial_reset()
{	
		/*(1)	沿复位方向转*/
		dial_motor_data[0].target_speed_rpm=-200;
		calculate_up_dial_motor_current_with_target_speed();
		
		/*(2)	堵转检测*/
		locked_rotor_detect(dial_motor_data[0].give_current,-700,20,&up_dial_start_reset_locked_rotor_flag,&up_dial_start_reset_locked_rotor_cnt);//上拨弹盘复位堵转检测
		/*(3)	刚检测到堵转*/
		if(up_dial_start_reset_locked_rotor_flag_last!=1 && up_dial_start_reset_locked_rotor_flag==1)
		{
			up_dial_reset_offset_total_ecd=dial_motor_data[0].total_ecd;//改偏置角度，以后每次转都要以当前位置为基点转固定角度
			dial_motor_data[0].target_total_ecd=dial_motor_data[0].total_ecd+Delta_Total_2006_Ecd_After_One_Round*2.0f/360.0f;//让它先定在当前角度
		}
		up_dial_start_reset_locked_rotor_flag_last=up_dial_start_reset_locked_rotor_flag;
}


/**	[1.2]							
  * brief         	上拨弹盘复位
	* postscript			上拨弹盘往复位方向转，并进行堵转检测；						*/

void up_dial_reset_and_Reset_Direction_locked_rotor_detect()
{
		/*(1)	沿复位方向转*/
		dial_motor_data[0].target_speed_rpm=-800;
		calculate_up_dial_motor_current_with_target_speed();
		
		/*(2)	上拨弹盘复位方向堵转检测*/
		locked_rotor_detect(dial_motor_data[0].give_current,-700,70,&up_dial_reset_locked_rotor_flag,&up_dial_reset_locked_rotor_cnt);//主程序里好像每次都有堵转检测
		
		/*(3)	如果上拨弹盘堵转了，表示复位完成*/
		if(up_dial_reset_locked_rotor_flag_last!=1 && up_dial_reset_locked_rotor_flag==1)
		{
			/*(3.1)	以当前复位位置为基准*/
			up_dial_rotate_cnt=0;
			up_dial_reset_offset_total_ecd=dial_motor_data[0].total_ecd;
			dial_motor_data[0].target_total_ecd=dial_motor_data[0].total_ecd;
			
			/*(3.2)	标志位清零，回归正常拨弹程序流程*/
			up_dial_dial_locked_rotor_flag=0;
			up_dial_reset_locked_rotor_flag=0;
		}
		
		up_dial_reset_locked_rotor_flag_last= up_dial_reset_locked_rotor_flag;

}

/**	[1.3]							
  * brief         	下拨弹盘一直顶着弹路，触发条件是上拨弹盘转到目标值附近
	* postscript			先速度环转到堵转，再位置环，目标值设置在堵转位置之后，一直把弹路顶紧							*/
uint8_t down_dial_on_or_off_flag=1;//下拨弹盘默认开着
int16_t KEY_Q;
int16_t KEY_Q_last;
int16_t KEY_E;
int16_t KEY_E_last;
void down_dial_rotate_until_locked_rotor()
{	
	
			KEY_Q=((rc_ctrl.key.v & KEY_PRESSED_OFFSET_Q) >> 6);
			KEY_E=((rc_ctrl.key.v & KEY_PRESSED_OFFSET_E) >> 7);//读取按键信息
			
			if(KEY_Q_last==0 && KEY_Q==1)
				down_dial_on_or_off_flag=1;//按一下Q下拨弹盘上紧
			if(KEY_E_last==0 && KEY_E==1)
				down_dial_on_or_off_flag=0;//按一下E下拨弹盘松劲
			
	
		/*(1)	上拨弹盘转到目标位置之后，根据是否堵转判断下拨弹盘的两种模式*/

			if(	(rc_ctrl.rc.s[0]==1 ||	rc_ctrl.rc.s[0]==3)	&&	down_dial_on_or_off_flag==1	&&	Game_Robot_State.mains_power_chassis_output==1 && Game_Robot_State.mains_power_shooter_output==1)//遥控器右侧拨杆拨到上面或中间，一直把位置环目标角度设成当前角度之后十度
			{
				dial_motor_data[1].target_total_ecd=dial_motor_data[1].total_ecd-Delta_Total_3508_Ecd_After_One_Round*10.0f/360.0f;
				calculate_down_dial_motor_current_with_target_total_angle();
			}			
			else
			{
//				dial_motor_data[1].target_speed_rpm=0;
//				calculate_down_dial_motor_current_with_target_speed();
				
				dial_motor_data[1].target_current=0;//2023.6.4,国赛名额争夺站打矿大第二局，英雄卡弹，怀疑是下拨弹盘卡了，把下面的速度环改成位置环松一松
				
			}
		
			KEY_Q_last=KEY_Q;
			KEY_E_last=KEY_E;
	
}

/**	[2]							
  * brief         	电机堵转检测函数
	* param[in]				要监测的电流，电流阈值，堵转累计次数阈值，
	* param[out]			堵转标志位变量，堵转计数变量
	* postscript										*/
void locked_rotor_detect(int16_t give_current,int16_t current_threshold,uint16_t locked_rotor_cnt_threshold,uint8_t *locked_rotor_flag,uint16_t *locked_rotor_cnt)
{
		/*(1)	判断电流有没有过大*/
	if(current_threshold >0)
	{
		if(give_current>current_threshold )//5500
				(*locked_rotor_cnt)++;//如果电流过大说明堵转	//++的优先级最高，所以得加括号
		else 
				if((*locked_rotor_cnt)	>0)	(*locked_rotor_cnt)--;
	}
	if(current_threshold <=0)
	{
		if(give_current<current_threshold )//5500
				(*locked_rotor_cnt)++;//如果电流过大说明堵转	//++的优先级最高，所以得加括号
		else 
				if((*locked_rotor_cnt)	>0)	(*locked_rotor_cnt)--;
	}
	
		/*(2)	判断电流有没有多次过大*/
		if((*locked_rotor_cnt)>locked_rotor_cnt_threshold)//如果多次检测到堵转，
		{
			(*locked_rotor_flag)=1;
			(*locked_rotor_cnt)=0;
		}
}

/**	[2]							
  * brief         	控制拨弹盘转固定角度
	* param[in]				拨弹盘转动的实际角度（角度制）
	* postscript										*/

uint8_t fric_target_speed_flag;//摩擦轮到达目标转速附近的标志变量

void up_dial_rotate_certain_angle(fp32 angle)
{
		/*(1)	如果遥控器拨到上面并且拨弹盘还没转过，设置转动的目标角度*/
//		if(	((rc_ctrl.rc.s[0]==1 && rc_ctrl_last1.rc.s[0]!=1)	||	(rc_ctrl.mouse.press_l!=0 && rc_ctrl_last1.mouse.press_l==0))	&&	fric_target_speed_flag==1	)//检测到拨杆拨上去的跳变沿,并且摩擦轮转到目标转速附近
//			if(	((rc_ctrl.rc.s[0]==1 && rc_ctrl_last.rc.s[0]!=1)	||	(rc_ctrl.mouse.press_l!=0 && rc_ctrl_last.mouse.press_l==0))	&& Game_Robot_State.remain_HP>0	 )
		if(	((rc_ctrl.rc.s[0]==1 && rc_ctrl_last.rc.s[0]!=1)	||	(rc_ctrl.mouse.press_l!=0 && rc_ctrl_last.mouse.press_l==0))	&& Game_Robot_State.mains_power_chassis_output==1 && Game_Robot_State.mains_power_shooter_output==1 )
		{
			up_dial_rotate_cnt++;//转动的总次数加一
			dial_motor_data[0].target_total_ecd=+up_dial_rotate_cnt*Delta_Total_2006_Ecd_After_One_Round*angle/360.0f+up_dial_reset_offset_total_ecd;//设置目标转动角度；加负号才能让拨弹盘正着转；用fp32数据类型存储最大是3.4*10的38次方，范围应该是够的，内存应该也是够的
		}
	
		calculate_up_dial_motor_current_with_target_total_angle();
}

/**	[]							
  * brief         	下拨弹盘转固定角度
	* param[in]				拨弹盘转动的目标角度（角度制）
	* postscript										*/
uint16_t down_dial_rotate_cnt=0;//下拨弹盘旋转次数
void down_dial_rotate_certain_angle(fp32 angle)
{
		if(	((rc_ctrl.rc.s[0]==1 && rc_ctrl_last.rc.s[0]!=1)	||	(rc_ctrl.mouse.press_l!=0 && rc_ctrl_last.mouse.press_l==0)))//之前多一个摩擦轮不到转速不能拨，后来因为换成pwm控制也读不到转速了
		{
			down_dial_rotate_cnt++;//转动的总次数加一
			dial_motor_data[1].target_total_ecd=-down_dial_rotate_cnt*Delta_Total_3508_Ecd_After_One_Round*(angle+0.0f)/360.0f;//设置目标转动角度；加负号才能让拨弹盘正着转；用fp32数据类型存储最大是3.4*10的38次方，范围应该是够的，内存应该也是够的

		}
	
		calculate_down_dial_motor_current_with_target_total_angle();
		
}




//uint8_t down_dial_locked_rotor_flag=0;//拨弹盘堵转标志位
//uint16_t down_dial_locked_rotor_cnt=0;//拨弹盘堵转计数
///**	[3]							测试程序
//  * brief         	拨弹盘一直转直到堵转转不动
//	* postscript			把弹一直拨到上拨弹盘弹夹内							*/
//void down_dial_rotate_until_locked_rotor()
//{	
//		if(down_dial_locked_rotor_flag==0 && rc_ctrl.rc.s[0]==1)//遥控器右侧拨杆拨到上面，开始检测堵转
//		{dial_motor_data[1].target_speed_rpm=-1000;	calculate_down_dial_motor_current_with_target_speed();}
//		
//		if(down_dial_locked_rotor_flag==1 && rc_ctrl.rc.s[0]==1)//遥控器右侧拨杆拨到上面，已经检测到堵转
//		calculate_down_dial_motor_current_with_target_total_angle();
//		
//		if(rc_ctrl.rc.s[0]==3)//遥控器拨杆往下拨，拨弹盘无力，堵转标志清零
//		{dial_motor_data[1].target_current=0;down_dial_locked_rotor_flag=0;}
//		
//		/*(3)	堵转检测*/
//		if(down_dial_locked_rotor_flag==0)
//		{
//			/*(3.1)	判断电流有没有过大*/
//		if(abs(dial_motor_data[1].give_current)>6000 )//13000
//				down_dial_locked_rotor_cnt++;//如果电流过大说明堵转
//		else 
//				if(down_dial_locked_rotor_cnt	>0)	down_dial_locked_rotor_cnt--;
//		
//			/*(3.2)	判断电流有没有多次过大*/
//		if(down_dial_locked_rotor_cnt>1000)//如果多次检测到堵转，拨弹盘目标速度给零，标志位置1，表示堵转了;堵转计数也清一下零吧，以便下次使用//2000
//		{
////			/*方案1：堵转之后定速度为零*/
////			dial_motor_data[1].target_speed_rpm=0;
////			calculate_down_dial_motor_current_with_target_speed();
//			
////			/*方案2：堵转之后定到当前位置*/
////			dial_motor_data[1].target_total_ecd=dial_motor_data[1].total_ecd;
////			calculate_down_dial_motor_current_with_target_total_angle();
//			
//				/*方案3：堵转之后定到当前位置再往前个角度，下拨弹盘一直处于堵转状态*/
//				dial_motor_data[1].target_total_ecd=dial_motor_data[1].total_ecd-Delta_Total_3508_Ecd_After_One_Round*45.0f/360.0f;//5.0f表示5°，后面加的一项是5°对应的编码器值
//				calculate_down_dial_motor_current_with_target_total_angle();
//			
//			down_dial_locked_rotor_flag=1;
//			down_dial_locked_rotor_cnt=0;
//		}			
//		
//		}
//}


/**	[4]							
  * brief         	摩擦轮以一定速度转动
	* postscript			加缓启动							*/
int16_t fric_soft_start_cnt=0;//如果阶梯速度还不够缓的话再用计数延长时间周期缓
uint16_t fric_soft_start_ladder=5;//摩擦轮缓启动阶梯速度
fp32 fric_ladder_target_speed[2]={0};//因为速度环的入口参数设置的是fp32，不然其实int16_t就够了
void fric_rotate_at_certain_speed(int16_t fric_motor_speed)
{
		if(rc_ctrl.rc.s[0]==1 || rc_ctrl.rc.s[0]==3)
		{
			fric_motor_data[0].target_speed_rpm=-fric_motor_speed;//右摩擦轮
			fric_motor_data[1].target_speed_rpm=+fric_motor_speed;//左摩擦轮
		}
		else 
		{
			fric_motor_data[0].target_speed_rpm=0;//右摩擦轮
			fric_motor_data[1].target_speed_rpm=0;//左摩擦轮
		}
		
		/* 设置摩擦轮缓启动阶梯转速 */
		fric_soft_start_cnt++;
		if(fric_soft_start_cnt==1)
		{
			for(uint8_t i=0;i<=1;i++)
			{
				if(fric_ladder_target_speed[i] <	fric_motor_data[i].target_speed_rpm) 
				fric_ladder_target_speed[i] += (fric_soft_start_ladder < fric_motor_data[i].target_speed_rpm - fric_ladder_target_speed[i]) ? fric_soft_start_ladder : fric_motor_data[i].target_speed_rpm - fric_ladder_target_speed[i];
				if(fric_ladder_target_speed[i] >	fric_motor_data[i].target_speed_rpm) 
				fric_ladder_target_speed[i] -= (fric_soft_start_ladder <  fric_ladder_target_speed[i] 	-	fric_motor_data[i].target_speed_rpm) ? fric_soft_start_ladder : fric_ladder_target_speed[i] - fric_motor_data[i].target_speed_rpm;
			}
			fric_soft_start_cnt=0;
		}
		
		fric_motor_data[0].target_speed_rpm=fric_ladder_target_speed[0];
		fric_motor_data[1].target_speed_rpm=fric_ladder_target_speed[1];
		calculate_fric_motor_current_with_target_speed();
	
		/*判断摩擦轮有没有达到目标转速*/
		if(	(abs(fric_motor_data[0].speed_rpm)>fric_motor_speed-200)	&& (abs(fric_motor_data[1].speed_rpm)>fric_motor_speed-200)	)
			fric_target_speed_flag=1;
		else 
			fric_target_speed_flag=0;
}

uint16_t fric_speed=2000;
uint16_t fric_ladder_speed=1000;
uint16_t fric_ladder=1;
uint16_t fric_soft_start_cnt_by_PWM;
void fric_rotate_by_PWM(uint16_t fric_target_speed)
{
		if((rc_ctrl.rc.s[0]==RC_SW_MID||rc_ctrl.rc.s[0]==RC_SW_UP))	
		fric_speed=fric_target_speed;
		else	
		fric_speed=1000;
	
		fric_soft_start_cnt_by_PWM++;
		if(fric_soft_start_cnt_by_PWM==1)
		{
		/*摩擦轮缓启动*/
		if(fric_ladder_speed < fric_speed) 
		fric_ladder_speed += (fric_ladder < fric_speed - fric_ladder_speed) ? fric_ladder : fric_speed - fric_ladder_speed;
		if(fric_ladder_speed > fric_speed) 
		fric_ladder_speed -= (fric_ladder <  fric_ladder_speed - fric_speed) ? fric_ladder : fric_ladder_speed - fric_speed;
		
		fric_soft_start_cnt_by_PWM=0;
		}
		
		TIM1->CCR1=fric_ladder_speed;
		TIM1->CCR2=fric_ladder_speed;

}

