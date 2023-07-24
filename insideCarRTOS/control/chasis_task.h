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
//底盘3508最大can发送电流值
#define MAX_MOTOR_CAN_CURRENT 16000.0f
//电机速度环PID
#define M3508_MOTOR_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M3508_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

void motor_init(void);
void limit(int16_t *target, int16_t min, int16_t max);

#endif
