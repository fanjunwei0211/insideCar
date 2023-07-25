#include "chasis_task.h"
#include "main.h"


extern int16_t probe_speed;

int16_t insideCar_speed;  //����С���ٶ����ñ���   ��Χ-16000~+16000
int16_t speed_l = 0,speed_r = 0;
first_order_filter_type_t motov_filter;
double distance = 0, distance_target = 0; 
PID_TypeDef motor_pid[5];  //3508���pid�ṹ��   1-2ǰ��  3-4��  5̽ͷ
PID_TypeDef distance_pid;
float pid_dis[3] = {10,0.2,0};
char car_mode = 2;  //С������ģʽ  0-->ʧ��ģʽ  1-->�ٶ�ģʽ   2-->λ��ģʽ




void Chasis_task(void const * argument)
{
		//3508���can��ʼ��
	motor_init();
	insideCar_speed = 0;
  for(;;)
	{
		osDelay(5);
		
		if(car_mode == 2)
		{
			//ͨ������������С��ǰ���ľ��� ang/19/8192*48 ��λmm
			distance = 1.0*moto_chassis[0].total_angle/155648*48*PI;
			distance_pid.target = distance_target;
			insideCar_speed = distance_pid.f_cal_pid(&distance_pid,distance);
		}

		//һ�׵�ͨ�˲���
		first_order_filter_cali(&motov_filter,insideCar_speed);
		
		//���̵���ٶ�����
		motor_pid[0].target = 48.0/65*motov_filter.out;
		motor_pid[1].target = -48.0/65*motov_filter.out;
		motor_pid[2].target = -motov_filter.out;
		motor_pid[3].target = motov_filter.out;
		
		motor_pid[4].target = probe_speed;
		limit(&speed_l,-4000,4000);
		limit(&speed_r,-4000,4000);
		TIM1->CCR2 = speed_l;
		TIM1->CCR3 = speed_r;
			
		for(int i=0; i<5; i++)
		{																							
			motor_pid[i].f_cal_pid(&motor_pid[i],moto_chassis[i].speed_rpm);    //�����趨ֵ����PID���㡣
		}
		
		if(car_mode != 0)
		{
			set_moto_current_CAN1(MotoHead,   motor_pid[0].output,   //��PID�ļ�����ͨ��CAN���͵����
																				motor_pid[1].output,
																				0,
																				0);

			set_moto_current_CAN2(MotoHead,   motor_pid[2].output,
																				motor_pid[3].output,
																				motor_pid[4].output,   //��PID�ļ�����ͨ��CAN���͵����
																				0);
		}
		else
		{
			set_moto_current_CAN1(MotoHead,   0,0,0,0);
			set_moto_current_CAN2(MotoHead,   0,0,0,0);
		}
	}
}

void motor_init()
{
	can_filter_init();
	first_order_filter_init(&motov_filter,0.004,0.33333333f);
  for(int i=0; i<5; i++)
  {	
    pid_init(&motor_pid[i]);
    motor_pid[i].f_param_init(&motor_pid[i],PID_Speed,15000,3000,10,0,8000,0,3,0.5,0);
  }
	pid_init(&distance_pid);
	distance_pid.f_param_init(&distance_pid,PID_Position,800,600,5,0,800,0,pid_dis[0],pid_dis[1],pid_dis[2]);
}

void limit(int16_t *target, int16_t min, int16_t max)
{
	if(*target > max)
		*target = max;
	if(*target < min)
		*target = min;
}
	