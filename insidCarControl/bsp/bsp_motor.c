#include "bsp_motor.h"
#include "main.h"


float angle_probe = 0;    //内爬小车旋转探头的转动角度
int16_t insideCar_speed;  //内爬小车速度设置变量   范围-16000~+16000
first_order_filter_type_t motov_filter;
PID_TypeDef motor_pid[5];  //3508电机pid结构体
//PID_TypeDef motor_pid_position;  //3508电机位置环pid结构体

const fp32 motor_speed_pid_param[3] = {M3508_MOTOR_SPEED_PID_KP, M3508_MOTOR_SPEED_PID_KI, M3508_MOTOR_SPEED_PID_KD};

void motor_init()
{
	//初始化3508电机PID 
	my_can_filter_init_recv_all(&hcan1);
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
	first_order_filter_init(&motov_filter,0.004,0.33333333f);
  for(int i=0; i<5; i++)
  {	
    pid_init(&motor_pid[i]);
    motor_pid[i].f_param_init(&motor_pid[i],PID_Speed,15000,3000,10,0,8000,0,3,0.5,0);
  }
//	pid_init(&motor_pid_position);
//	motor_pid_position.f_param_init(&motor_pid_position,PID_Position,4000,1000,500,0,5000,0,0.01,0.002,0);
}

	
