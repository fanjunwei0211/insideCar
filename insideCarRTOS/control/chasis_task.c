#include "chasis_task.h"
#include "main.h"


extern int16_t probe_speed;

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
}

	


void Chasis_task(void const * argument)
{
		//3508���can��ʼ��
	motor_init();
	insideCar_speed = 1140;
  for(;;)
	{
		osDelay(5);

		first_order_filter_cali(&motov_filter,insideCar_speed);
		motor_pid[0].target = 25.0/33*motov_filter.out;
		motor_pid[1].target = -25.0/33*motov_filter.out;
		motor_pid[2].target = motov_filter.out;
		motor_pid[3].target = -motov_filter.out;
		
		motor_pid[4].target = probe_speed;
			
		for(int i=0; i<5; i++)
		{																							
			motor_pid[i].f_cal_pid(&motor_pid[i],moto_chassis[i].speed_rpm);    //�����趨ֵ����PID���㡣
		}
		set_moto_current(&hcan1, MotoHead,  motor_pid[0].output,   //��PID�ļ�����ͨ��CAN���͵����
																				motor_pid[1].output,
																				motor_pid[2].output,
																				motor_pid[3].output);

		set_moto_current(&hcan1, MotoTail,  motor_pid[4].output,   //��PID�ļ�����ͨ��CAN���͵����
																				0,
																				0,
																				0);
	}
}
