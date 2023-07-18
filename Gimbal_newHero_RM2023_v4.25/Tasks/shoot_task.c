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
#include "stdlib.h"//Ҫ���þ���ֵ��������Ħ���ֵ����ж��Ǳ�
#include "Referee.h"
#include "pitch_motor.h"
#include "aim_motor.h"
#include "Nmanifold_usart_task.h"


#define ABS(x) ( (x)>0?(x):-(x) )

/*Ħ����canͨ��pid����*/
//#define FRIC_SPEED_10 4800
//#define FRIC_SPEED_16 6000
float FRIC_SPEED_10=3000;
float FRIC_SPEED_16=4000;
/*Ħ����pwm����*/
#define fric_0m 1000//1500
//#define fric_10m 1400//1500
//#define fric_16m 1540//1510
uint16_t fric_10m=1475;//1400
uint16_t fric_14m=1670 ;//1530,1475�ȶ�14.0��0.1��1480�ȶ�14.4��0.2
uint16_t fric_15m=1500;//���ٻ�ûУ���������������
int16_t fric_test=1000;
uint16_t fric_target=1475;
/*****3508�����궨��*****/
#define Delta_3508_Ecd_After_One_Round 8192			//3508תһȦ֮���������ֵ�ı���
#define Reduction_Ratio_of_3508_Motor 3591/187		//3508����ļ��ٱ�(�궨������ֲ������㣬���治�ӷֺţ���Ȼ�˷�����ʱ�ᱻ����ָ��)
#define Delta_Total_3508_Ecd_After_One_Round Delta_3508_Ecd_After_One_Round*Reduction_Ratio_of_3508_Motor//�óɺ궨����⾫����ʧ
/*****2006�����궨��*****/
#define Delta_2006_Ecd_After_One_Round 8192			//3508תһȦ֮���������ֵ�ı���
#define Reduction_Ratio_of_2006_Motor 36		//3508����ļ��ٱ�(�궨������ֲ������㣬���治�ӷֺţ���Ȼ�˷�����ʱ�ᱻ����ָ��)
#define Delta_Total_2006_Ecd_After_One_Round Delta_2006_Ecd_After_One_Round*Reduction_Ratio_of_2006_Motor//�óɺ궨����⾫����ʧ

/*****extern�����Ϣ*****/
extern motor_data_t 		dial_motor_data[2];
extern motor_data_t 		fric_motor_data[2];
extern motor4310_data_t motor4310_data[8];
extern motor_data_t aim_motor_data;
extern ext_game_robot_state_t Game_Robot_State;
ext_game_robot_state_t Game_Robot_State_last;
/*****�ϲ�����ת�̶��Ƕ���ر���*****/
int64_t up_dial_reset_offset_total_ecd=0;			 //�ϲ����̸�λƫ���ܱ������Ƕȣ���λƫ���ܱ������Ƕȣ�Ҳ���Ǵ�����ܱ������ǶȻ�����ת�̶��Ƕ�
uint16_t up_dial_rotate_cnt=0;//�ϲ�������ת����

/*****���²����̵Ķ�ת��־�����Ͷ�ת����*****/
uint8_t up_dial_start_reset_locked_rotor_flag=0;//�ϲ�����	���ϵ�	��λ����	��ת��־����
uint8_t up_dial_start_reset_locked_rotor_flag_last;
uint16_t up_dial_start_reset_locked_rotor_cnt=0;//�ϲ�����	���ϵ�	��λ����	��ת����

uint8_t up_dial_reset_locked_rotor_flag=0;			//�ϲ�����					��λ����	��ת��־����
uint8_t up_dial_reset_locked_rotor_flag_last=0;//�ϲ����̸�λ�����ת��־����
uint16_t up_dial_reset_locked_rotor_cnt=0;			//�ϲ�����					��λ����	��ת����

uint8_t up_dial_dial_locked_rotor_flag=0;				//�ϲ�����					��������	��ת��־λ
uint16_t up_dial_dial_locked_rotor_cnt=0;				//�ϲ�����					��������	��ת����

uint8_t down_dial_locked_rotor_flag=0;					//�²�����										��ת��־����
uint8_t down_dial_locked_rotor_flag_last;	
uint16_t down_dial_locked_rotor_cnt=0;					//�²�����										��ת����

RC_ctrl_t rc_ctrl_last;
int16_t KEY_CTRL;
int16_t KEY_CTRL_last;
uint8_t fric_duandian_flag;
uint16_t fric_duandian_cnt;
uint8_t fric_shangdian_flag;
uint8_t fric_shangdian_flag_last;
uint16_t fric_shangdian_cnt;

uint8_t fric_target_init_flag;//����ϵͳ����������16m/s��ʱ����Ϊ��ctrlҪ�л��ٶȣ����Ե��ȳ�ʼ��һ�����٣���Ϊֻ��Ҫ��ʼ��һ�Σ����������־λ��ʼ�������һ��Ҳ������
uint16_t fric_UI_flag;

extern AutoAim_Data_Rx AutoAim_Data_Receive;
AutoAim_Data_Rx AutoAim_Data_Receive_last;
uint16_t autoaim_shoot_allowed_number;
void shoot_task(void const * argument)
{
	fric_motor_init();//Ħ����������Ϣ�Ϳ�����Ϣ��ʼ��
	dial_motor_init();
	aim_motor_init();
	while(1)
	{
		Aim_Motor_Control();//��׼������
		
		/*(1)	����Ħ����*/
		/*pwm����*/
		KEY_CTRL=((rc_ctrl.key.v & KEY_PRESSED_OFFSET_CTRL) >> 5);//��ȡ����
		
			/*Ĭ�ϵ������ã����ݲ���ϵͳ��*/
			if(Game_Robot_State.shooter_id1_42mm_speed_limit==16)		
			{
				if(fric_target_init_flag==0)
				{
					fric_target=fric_14m;
					fric_target_init_flag=1;
				}
				
				if(KEY_CTRL_last==0 && KEY_CTRL==1)//��crtl�л�����
					fric_target=	(fric_target==fric_14m)?fric_15m:fric_14m;
			}
			else 
				fric_target=fric_10m;
			
			if(Game_Robot_State_last.mains_power_shooter_output==1 && Game_Robot_State.mains_power_shooter_output==0)//�����������µ����ϵ磬���������ϵ繤����pwm����1000һ��ʱ�䣬��Ȼ����������
			{
				fric_duandian_flag=1;//��Ħ�����ϵ籣��ģʽ��һ��ʼ�ȳ���һ��ʱ��pwm1000״̬
			}
			if(Game_Robot_State_last.mains_power_shooter_output==0 && Game_Robot_State.mains_power_shooter_output==1)//�����������µ����ϵ磬���������ϵ繤����pwm����1000һ��ʱ�䣬��Ȼ����������
			{
				fric_shangdian_flag=1;//��Ħ�����ϵ籣��ģʽ��һ��ʼ�ȳ���һ��ʱ��pwm1000״̬
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
			
			if(fric_shangdian_flag_last==1 &&	fric_shangdian_flag==0)//�����µ����ϵ�Ħ����ת�ٲ����µ�bug
				fric_target_init_flag=0;
		
			if(fric_target!=1000 && Game_Robot_State.mains_power_shooter_output==1 && (rc_ctrl.rc.s[0]==1 || rc_ctrl.rc.s[0]==3)	)fric_UI_flag=1;
			else fric_UI_flag=0;
			
			
			fric_rotate_by_PWM(fric_target);
			KEY_CTRL_last=KEY_CTRL;
			fric_shangdian_flag_last=fric_shangdian_flag;
//		
//		TIM1->CCR1=fric_test;
//		TIM1->CCR2=fric_test;
		
		/*can����*/
//		if(motor4310_data[0].angle_actual_float>50)//pitcḩ����������תĦ����
//		{
//			if(Game_Robot_State.shooter_id1_42mm_speed_limit==16)		
//				fric_rotate_at_certain_speed(FRIC_SPEED_16);
//			else 
//				fric_rotate_at_certain_speed(FRIC_SPEED_10);
//		}
//		else
//			fric_rotate_at_certain_speed(0);

		
		/*(2)	����������*/
		
		
		
		/*(2.1)	�����̶�ת���*/
		locked_rotor_detect(dial_motor_data[0].give_current,700,70,&up_dial_dial_locked_rotor_flag,&up_dial_dial_locked_rotor_cnt);//�ϲ����̲�����ת���

		
		/*�����¼������ʱ��bug*/
		if(Game_Robot_State.mains_power_chassis_output==0)//������chassis�ӿڶ���
			up_dial_start_reset_locked_rotor_flag=0;//�����ϵ縴λ
		
		/*(2.2)	�ϲ������ϵ縴λ*/
		if(up_dial_start_reset_locked_rotor_flag==0)
			up_dial_reset();
		
		/*(2.3)	���²�������ϴ�*/
		else
		{			

			if(rc_ctrl.mouse.press_r==0)//���û��������Ҽ�����������
			{	
					/*(2.3.1)	����ϲ���������������û�з�����ת*/
					if(up_dial_dial_locked_rotor_flag==0)
					{
						up_dial_rotate_certain_angle(120.0f);//�ϲ�����ת�̶��Ƕ�
						down_dial_rotate_until_locked_rotor();//�ϲ�����ת��Ŀ��λ��֮���²�����ת����ת
					}
					
					
					/*(2.3.2)	����ϲ����̶�ת*/
					if(up_dial_dial_locked_rotor_flag==1)
					{
						up_dial_reset_and_Reset_Direction_locked_rotor_detect();
						down_dial_rotate_until_locked_rotor();//�ϲ�����ת��Ŀ��λ��֮���²�����ת����ת		//����Ҫ��Ҫ����仰����Ϊ���û��ת��Ŀ��λ���²������ƺ�Ҳ���ᶯ			
					}
				
			}
			else//			�����������Ҽ�����������ģʽ																											����󲦸˲��������棬������ģʽ
			{
				
				if(rc_ctrl_last.mouse.press_r==0 && rc_ctrl.mouse.press_r==1)//����յ�����Ҽ���������ģʽ�����һ����������
						autoaim_shoot_allowed_number=0;
				
//				if(rc_ctrl_last.rc.s[0]!=1 && rc_ctrl.rc.s[0]==1)//ֻ��ң�����������������   �����⵽�Ҳ��˴��м䲦�������������
				if(rc_ctrl_last.mouse.press_l==0 && rc_ctrl.mouse.press_l==1)
						autoaim_shoot_allowed_number=1;//��������++
				
				if(AutoAim_Data_Receive_last.Shoot_Freq==0 && AutoAim_Data_Receive.Shoot_Freq>0)//����Ӿ��������ܴ򵯵�������
					if(autoaim_shoot_allowed_number>0)//�����������������
					{
						up_dial_rotate_cnt++;//ת�����ܴ�����һ
						dial_motor_data[0].target_total_ecd=+up_dial_rotate_cnt*Delta_Total_2006_Ecd_After_One_Round*120.0f/360.0f+up_dial_reset_offset_total_ecd;//����Ŀ��ת���Ƕȣ��Ӹ��Ų����ò���������ת����fp32�������ʹ洢�����3.4*10��38�η�����ΧӦ���ǹ��ģ��ڴ�Ӧ��Ҳ�ǹ���
					
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
		send_motor_3508current_through_CAN2(fric_motor_data[0].target_current,fric_motor_data[1].target_current,aim_motor_data.target_current,0);//������Ħ���ֵĵ���
		vTaskDelay(2);
		send_motor_3508_or_2006_current_through_CAN1(dial_motor_data[1].target_current,dial_motor_data[0].target_current,0,0);//�����������̵ĵ���
		vTaskDelay(2);
		
		/*vofa��ʾ����(���������ļ���Ҳ������Ȼbusy)*/
		//send_data_to_vofa4(dial_motor_data[1].speed_rpm,dial_motor_data[1].target_speed_rpm,10,0);
	}
	
}


/**	[1.1]							
  * brief         	���ϵ磬�ϲ����̸�λ
	* postscript			�ϲ���������λ����ת�������ж�ת���							*/
void up_dial_reset()
{	
		/*(1)	�ظ�λ����ת*/
		dial_motor_data[0].target_speed_rpm=-200;
		calculate_up_dial_motor_current_with_target_speed();
		
		/*(2)	��ת���*/
		locked_rotor_detect(dial_motor_data[0].give_current,-700,20,&up_dial_start_reset_locked_rotor_flag,&up_dial_start_reset_locked_rotor_cnt);//�ϲ����̸�λ��ת���
		/*(3)	�ռ�⵽��ת*/
		if(up_dial_start_reset_locked_rotor_flag_last!=1 && up_dial_start_reset_locked_rotor_flag==1)
		{
			up_dial_reset_offset_total_ecd=dial_motor_data[0].total_ecd;//��ƫ�ýǶȣ��Ժ�ÿ��ת��Ҫ�Ե�ǰλ��Ϊ����ת�̶��Ƕ�
			dial_motor_data[0].target_total_ecd=dial_motor_data[0].total_ecd+Delta_Total_2006_Ecd_After_One_Round*2.0f/360.0f;//�����ȶ��ڵ�ǰ�Ƕ�
		}
		up_dial_start_reset_locked_rotor_flag_last=up_dial_start_reset_locked_rotor_flag;
}


/**	[1.2]							
  * brief         	�ϲ����̸�λ
	* postscript			�ϲ���������λ����ת�������ж�ת��⣻						*/

void up_dial_reset_and_Reset_Direction_locked_rotor_detect()
{
		/*(1)	�ظ�λ����ת*/
		dial_motor_data[0].target_speed_rpm=-800;
		calculate_up_dial_motor_current_with_target_speed();
		
		/*(2)	�ϲ����̸�λ�����ת���*/
		locked_rotor_detect(dial_motor_data[0].give_current,-700,70,&up_dial_reset_locked_rotor_flag,&up_dial_reset_locked_rotor_cnt);//�����������ÿ�ζ��ж�ת���
		
		/*(3)	����ϲ����̶�ת�ˣ���ʾ��λ���*/
		if(up_dial_reset_locked_rotor_flag_last!=1 && up_dial_reset_locked_rotor_flag==1)
		{
			/*(3.1)	�Ե�ǰ��λλ��Ϊ��׼*/
			up_dial_rotate_cnt=0;
			up_dial_reset_offset_total_ecd=dial_motor_data[0].total_ecd;
			dial_motor_data[0].target_total_ecd=dial_motor_data[0].total_ecd;
			
			/*(3.2)	��־λ���㣬�ع�����������������*/
			up_dial_dial_locked_rotor_flag=0;
			up_dial_reset_locked_rotor_flag=0;
		}
		
		up_dial_reset_locked_rotor_flag_last= up_dial_reset_locked_rotor_flag;

}

/**	[1.3]							
  * brief         	�²�����һֱ���ŵ�·�������������ϲ�����ת��Ŀ��ֵ����
	* postscript			���ٶȻ�ת����ת����λ�û���Ŀ��ֵ�����ڶ�תλ��֮��һֱ�ѵ�·����							*/
uint8_t down_dial_on_or_off_flag=1;//�²�����Ĭ�Ͽ���
int16_t KEY_Q;
int16_t KEY_Q_last;
int16_t KEY_E;
int16_t KEY_E_last;
void down_dial_rotate_until_locked_rotor()
{	
	
			KEY_Q=((rc_ctrl.key.v & KEY_PRESSED_OFFSET_Q) >> 6);
			KEY_E=((rc_ctrl.key.v & KEY_PRESSED_OFFSET_E) >> 7);//��ȡ������Ϣ
			
			if(KEY_Q_last==0 && KEY_Q==1)
				down_dial_on_or_off_flag=1;//��һ��Q�²������Ͻ�
			if(KEY_E_last==0 && KEY_E==1)
				down_dial_on_or_off_flag=0;//��һ��E�²������ɾ�
			
	
		/*(1)	�ϲ�����ת��Ŀ��λ��֮�󣬸����Ƿ��ת�ж��²����̵�����ģʽ*/

			if(	(rc_ctrl.rc.s[0]==1 ||	rc_ctrl.rc.s[0]==3)	&&	down_dial_on_or_off_flag==1	&&	Game_Robot_State.mains_power_chassis_output==1 && Game_Robot_State.mains_power_shooter_output==1)//ң�����Ҳದ�˲���������м䣬һֱ��λ�û�Ŀ��Ƕ���ɵ�ǰ�Ƕ�֮��ʮ��
			{
				dial_motor_data[1].target_total_ecd=dial_motor_data[1].total_ecd-Delta_Total_3508_Ecd_After_One_Round*10.0f/360.0f;
				calculate_down_dial_motor_current_with_target_total_angle();
			}			
			else
			{
//				dial_motor_data[1].target_speed_rpm=0;
//				calculate_down_dial_motor_current_with_target_speed();
				
				dial_motor_data[1].target_current=0;//2023.6.4,������������վ����ڶ��֣�Ӣ�ۿ������������²����̿��ˣ���������ٶȻ��ĳ�λ�û���һ��
				
			}
		
			KEY_Q_last=KEY_Q;
			KEY_E_last=KEY_E;
	
}

/**	[2]							
  * brief         	�����ת��⺯��
	* param[in]				Ҫ���ĵ�����������ֵ����ת�ۼƴ�����ֵ��
	* param[out]			��ת��־λ��������ת��������
	* postscript										*/
void locked_rotor_detect(int16_t give_current,int16_t current_threshold,uint16_t locked_rotor_cnt_threshold,uint8_t *locked_rotor_flag,uint16_t *locked_rotor_cnt)
{
		/*(1)	�жϵ�����û�й���*/
	if(current_threshold >0)
	{
		if(give_current>current_threshold )//5500
				(*locked_rotor_cnt)++;//�����������˵����ת	//++�����ȼ���ߣ����Եü�����
		else 
				if((*locked_rotor_cnt)	>0)	(*locked_rotor_cnt)--;
	}
	if(current_threshold <=0)
	{
		if(give_current<current_threshold )//5500
				(*locked_rotor_cnt)++;//�����������˵����ת	//++�����ȼ���ߣ����Եü�����
		else 
				if((*locked_rotor_cnt)	>0)	(*locked_rotor_cnt)--;
	}
	
		/*(2)	�жϵ�����û�ж�ι���*/
		if((*locked_rotor_cnt)>locked_rotor_cnt_threshold)//�����μ�⵽��ת��
		{
			(*locked_rotor_flag)=1;
			(*locked_rotor_cnt)=0;
		}
}

/**	[2]							
  * brief         	���Ʋ�����ת�̶��Ƕ�
	* param[in]				������ת����ʵ�ʽǶȣ��Ƕ��ƣ�
	* postscript										*/

uint8_t fric_target_speed_flag;//Ħ���ֵ���Ŀ��ת�ٸ����ı�־����

void up_dial_rotate_certain_angle(fp32 angle)
{
		/*(1)	���ң�����������沢�Ҳ����̻�ûת��������ת����Ŀ��Ƕ�*/
//		if(	((rc_ctrl.rc.s[0]==1 && rc_ctrl_last1.rc.s[0]!=1)	||	(rc_ctrl.mouse.press_l!=0 && rc_ctrl_last1.mouse.press_l==0))	&&	fric_target_speed_flag==1	)//��⵽���˲���ȥ��������,����Ħ����ת��Ŀ��ת�ٸ���
//			if(	((rc_ctrl.rc.s[0]==1 && rc_ctrl_last.rc.s[0]!=1)	||	(rc_ctrl.mouse.press_l!=0 && rc_ctrl_last.mouse.press_l==0))	&& Game_Robot_State.remain_HP>0	 )
		if(	((rc_ctrl.rc.s[0]==1 && rc_ctrl_last.rc.s[0]!=1)	||	(rc_ctrl.mouse.press_l!=0 && rc_ctrl_last.mouse.press_l==0))	&& Game_Robot_State.mains_power_chassis_output==1 && Game_Robot_State.mains_power_shooter_output==1 )
		{
			up_dial_rotate_cnt++;//ת�����ܴ�����һ
			dial_motor_data[0].target_total_ecd=+up_dial_rotate_cnt*Delta_Total_2006_Ecd_After_One_Round*angle/360.0f+up_dial_reset_offset_total_ecd;//����Ŀ��ת���Ƕȣ��Ӹ��Ų����ò���������ת����fp32�������ʹ洢�����3.4*10��38�η�����ΧӦ���ǹ��ģ��ڴ�Ӧ��Ҳ�ǹ���
		}
	
		calculate_up_dial_motor_current_with_target_total_angle();
}

/**	[]							
  * brief         	�²�����ת�̶��Ƕ�
	* param[in]				������ת����Ŀ��Ƕȣ��Ƕ��ƣ�
	* postscript										*/
uint16_t down_dial_rotate_cnt=0;//�²�������ת����
void down_dial_rotate_certain_angle(fp32 angle)
{
		if(	((rc_ctrl.rc.s[0]==1 && rc_ctrl_last.rc.s[0]!=1)	||	(rc_ctrl.mouse.press_l!=0 && rc_ctrl_last.mouse.press_l==0)))//֮ǰ��һ��Ħ���ֲ���ת�ٲ��ܲ���������Ϊ����pwm����Ҳ������ת����
		{
			down_dial_rotate_cnt++;//ת�����ܴ�����һ
			dial_motor_data[1].target_total_ecd=-down_dial_rotate_cnt*Delta_Total_3508_Ecd_After_One_Round*(angle+0.0f)/360.0f;//����Ŀ��ת���Ƕȣ��Ӹ��Ų����ò���������ת����fp32�������ʹ洢�����3.4*10��38�η�����ΧӦ���ǹ��ģ��ڴ�Ӧ��Ҳ�ǹ���

		}
	
		calculate_down_dial_motor_current_with_target_total_angle();
		
}




//uint8_t down_dial_locked_rotor_flag=0;//�����̶�ת��־λ
//uint16_t down_dial_locked_rotor_cnt=0;//�����̶�ת����
///**	[3]							���Գ���
//  * brief         	������һֱתֱ����תת����
//	* postscript			�ѵ�һֱ�����ϲ����̵�����							*/
//void down_dial_rotate_until_locked_rotor()
//{	
//		if(down_dial_locked_rotor_flag==0 && rc_ctrl.rc.s[0]==1)//ң�����Ҳದ�˲������棬��ʼ����ת
//		{dial_motor_data[1].target_speed_rpm=-1000;	calculate_down_dial_motor_current_with_target_speed();}
//		
//		if(down_dial_locked_rotor_flag==1 && rc_ctrl.rc.s[0]==1)//ң�����Ҳದ�˲������棬�Ѿ���⵽��ת
//		calculate_down_dial_motor_current_with_target_total_angle();
//		
//		if(rc_ctrl.rc.s[0]==3)//ң�����������²�����������������ת��־����
//		{dial_motor_data[1].target_current=0;down_dial_locked_rotor_flag=0;}
//		
//		/*(3)	��ת���*/
//		if(down_dial_locked_rotor_flag==0)
//		{
//			/*(3.1)	�жϵ�����û�й���*/
//		if(abs(dial_motor_data[1].give_current)>6000 )//13000
//				down_dial_locked_rotor_cnt++;//�����������˵����ת
//		else 
//				if(down_dial_locked_rotor_cnt	>0)	down_dial_locked_rotor_cnt--;
//		
//			/*(3.2)	�жϵ�����û�ж�ι���*/
//		if(down_dial_locked_rotor_cnt>1000)//�����μ�⵽��ת��������Ŀ���ٶȸ��㣬��־λ��1����ʾ��ת��;��ת����Ҳ��һ����ɣ��Ա��´�ʹ��//2000
//		{
////			/*����1����ת֮���ٶ�Ϊ��*/
////			dial_motor_data[1].target_speed_rpm=0;
////			calculate_down_dial_motor_current_with_target_speed();
//			
////			/*����2����ת֮�󶨵���ǰλ��*/
////			dial_motor_data[1].target_total_ecd=dial_motor_data[1].total_ecd;
////			calculate_down_dial_motor_current_with_target_total_angle();
//			
//				/*����3����ת֮�󶨵���ǰλ������ǰ���Ƕȣ��²�����һֱ���ڶ�ת״̬*/
//				dial_motor_data[1].target_total_ecd=dial_motor_data[1].total_ecd-Delta_Total_3508_Ecd_After_One_Round*45.0f/360.0f;//5.0f��ʾ5�㣬����ӵ�һ����5���Ӧ�ı�����ֵ
//				calculate_down_dial_motor_current_with_target_total_angle();
//			
//			down_dial_locked_rotor_flag=1;
//			down_dial_locked_rotor_cnt=0;
//		}			
//		
//		}
//}


/**	[4]							
  * brief         	Ħ������һ���ٶ�ת��
	* postscript			�ӻ�����							*/
int16_t fric_soft_start_cnt=0;//��������ٶȻ��������Ļ����ü����ӳ�ʱ�����ڻ�
uint16_t fric_soft_start_ladder=5;//Ħ���ֻ����������ٶ�
fp32 fric_ladder_target_speed[2]={0};//��Ϊ�ٶȻ�����ڲ������õ���fp32����Ȼ��ʵint16_t�͹���
void fric_rotate_at_certain_speed(int16_t fric_motor_speed)
{
		if(rc_ctrl.rc.s[0]==1 || rc_ctrl.rc.s[0]==3)
		{
			fric_motor_data[0].target_speed_rpm=-fric_motor_speed;//��Ħ����
			fric_motor_data[1].target_speed_rpm=+fric_motor_speed;//��Ħ����
		}
		else 
		{
			fric_motor_data[0].target_speed_rpm=0;//��Ħ����
			fric_motor_data[1].target_speed_rpm=0;//��Ħ����
		}
		
		/* ����Ħ���ֻ���������ת�� */
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
	
		/*�ж�Ħ������û�дﵽĿ��ת��*/
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
		/*Ħ���ֻ�����*/
		if(fric_ladder_speed < fric_speed) 
		fric_ladder_speed += (fric_ladder < fric_speed - fric_ladder_speed) ? fric_ladder : fric_speed - fric_ladder_speed;
		if(fric_ladder_speed > fric_speed) 
		fric_ladder_speed -= (fric_ladder <  fric_ladder_speed - fric_speed) ? fric_ladder : fric_ladder_speed - fric_speed;
		
		fric_soft_start_cnt_by_PWM=0;
		}
		
		TIM1->CCR1=fric_ladder_speed;
		TIM1->CCR2=fric_ladder_speed;

}

