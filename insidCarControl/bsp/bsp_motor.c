#include "bsp_motor.h"
#include "main.h"


float angle_probe = 0;    //����С����ת̽ͷ��ת���Ƕ�
int16_t insideCar_speed;  //����С���ٶ����ñ���   ��Χ-16000~+16000
first_order_filter_type_t motov_filter;
PID_TypeDef motor_pid[5];  //3508���pid�ṹ��
//PID_TypeDef motor_pid_position;  //3508���λ�û�pid�ṹ��

const fp32 motor_speed_pid_param[3] = {M3508_MOTOR_SPEED_PID_KP, M3508_MOTOR_SPEED_PID_KI, M3508_MOTOR_SPEED_PID_KD};

void motor_init()
{
	//��ʼ��3508���PID 
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

	
