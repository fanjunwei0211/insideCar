#ifndef CHASIS_TASK_H
#define CHASIS_TASK_H
#include "struct_typedef.h"
#include "cmsis_os.h"
#include "pid.h"
#include "bsp_can.h"
#include "user_lib.h"

extern int16_t insideCar_speed;
extern first_order_filter_type_t motov_filter;
extern PID_TypeDef motor_pid[]; 
//extern PID_TypeDef motor_pid_position; 
//����3508���can���͵���ֵ
#define MAX_MOTOR_CAN_CURRENT 16000.0f
//����ٶȻ�PID
#define M3508_MOTOR_SPEED_PID_KP 3.0f
#define M3508_MOTOR_SPEED_PID_KI 0.5f
#define M3508_MOTOR_SPEED_PID_KD 0.0f
#define M3508_MOTOR_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M3508_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

void motor_init(void);

#endif
