#include "gimbal_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "yaw_motor.h"
#include "remote_control.h"
#include "motor_can.h"
#include "pitch_motor.h"
#include "vofa.h"
#include "yaw_motor.h"
#include "can_transmit.h"

#include "pid.h"
#include "servo.h"
#include "BMI088driver.h"
#include "math.h"

#include "music.h"
#include "Nmanifold_usart_task.h"
#include "dial_motor.h"
extern motor_data_t 	yaw_motor_data;
extern motor_data_t 	dial_motor_data[2];
extern float yaw_INS_speed;				//yaw轴陀螺仪速度
extern float yaw_INS_angle;				//yaw轴陀螺仪角度
extern float yaw_target_INS_speed;	//yaw轴陀螺仪目标速度
extern float yaw_target_INS_angle;	//yaw轴陀螺仪目标角度
extern fp32 INS_angle_deg[3];//欧拉角(degree)
//extern float pitch_target_INS_angle;		//pitch轴陀螺仪目标角度
extern pid_type_def yaw_motor_INS_angle_pid;
extern float yaw_autoaim_target_INS_speed;	//yaw轴陀螺仪目标速度
extern motor4310_data_t motor4310_data[8]; //后面那个接收电机数据的函数要用
extern float yaw_target_encoder_speed;				//yaw轴编码器目标速度
extern AutoAim_Data_Rx AutoAim_Data_Receive;
extern float pitch_autoaim_target_INS_angle;		//pitch轴陀螺仪目标角度
extern float yaw_autoaim_target_INS_angle;	//yaw轴陀螺仪目标角度
extern motor_data_t 		fric_motor_data[2];				//存放电机数据信息
extern float yaw_target_encoder_speed;			//yaw轴编码器目标速度
extern float yaw_encoder_speed;						//yaw轴编码器速度
extern float yaw_encoder_angle;						//yaw轴编码器角度
extern float yaw_target_encoder_angle;			//yaw轴编码器目标角度
extern  float pitch_target_INS_angle;						//pitch轴陀螺仪目标角度
extern float pitch_INS_speed;									//pitch轴陀螺仪角度
extern float pitch_target_INS_speed;						//pitch轴陀螺仪目标速度
extern int16_t rc_mouse_y;
extern uint16_t up_dial_start_reset_locked_rotor_cnt;
extern uint8_t up_dial_start_reset_locked_rotor_flag;
float INS_roll_for_UI;
extern fp32 INS_angle[3];      //euler angle, unit rad.欧拉角 单位 rad
extern float pitch_INS_angle;	
extern motor_data_t 	aim_motor_data;		
void gimbal_task(void const * argument)
{
	yaw_motor_init();
	pitch_motor_init();//pitch轴用上陀螺仪作位置环的时候才有东西
	
//		play();//播放开机音乐
	while(1)
	{
		Yaw_Motor_Control();
		Pitch_Motor_Control();//换头要重新标一下陀螺仪的限位角度
		
		INS_roll_for_UI=INS_angle[1];
	
		send_motor_3508current_or_6020voltage_through_CAN1(yaw_motor_data.target_current,0,0,0);	
		
		vTaskDelay(2);
		
		/*vofa显示波形(别在其他文件里也发，不然busy)*/
//		send_data_to_vofa4(yaw_target_encoder_speed,yaw_motor_data.speed_rpm,5908,yaw_motor_data.ecd);
//		send_data_to_vofa4(pitch_autoaim_target_INS_angle,INS_angle_deg[2],yaw_autoaim_target_INS_angle,INS_angle_deg[0]);
//		send_data_to_vofa4(fric_motor_data[0].speed_rpm,fric_motor_data[0].target_speed_rpm ,fric_motor_data[1].speed_rpm,fric_motor_data[1].target_speed_rpm);
//		send_data_to_vofa4(yaw_encoder_speed,yaw_target_encoder_speed,yaw_encoder_angle, yaw_target_encoder_angle);
//		send_data_to_vofa4(INS_angle_deg[2],pitch_target_INS_angle,pitch_INS_speed, rc_mouse_y);
//		send_data_to_vofa4(dial_motor_data[0].give_current,dial_motor_data[0].total_ecd,dial_motor_data[0].target_total_ecd,up_dial_start_reset_locked_rotor_flag);
//		send_data_to_vofa4(rc_ctrl.mouse.z,pitch_target_INS_angle,0,0);
//		send_data_to_vofa4(yaw_target_INS_speed,yaw_INS_speed,rc_ctrl.mouse.x,INS_angle_deg[0]);
//		send_data_to_vofa4(pitch_target_INS_speed,pitch_INS_speed,pitch_target_INS_angle, pitch_INS_angle);
		send_data_to_vofa4(aim_motor_data.target_speed_rpm,aim_motor_data.speed_rpm,aim_motor_data.target_total_ecd,aim_motor_data.total_ecd);
		vTaskDelay(2);
	}
}


