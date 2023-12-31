#ifndef BSP_MOTOR_H
#define BSP_MOTOR_H
#include "struct_typedef.h"
#include "pid.h"
#include "bsp_can.h"
#include "user_lib.h"

extern int16_t insideCar_speed;
extern float angle_probe;
extern first_order_filter_type_t motov_filter;
extern PID_TypeDef motor_pid[]; 
//extern PID_TypeDef motor_pid_position; 
//底盘3508最大can发送电流值
#define MAX_MOTOR_CAN_CURRENT 16000.0f
//电机速度环PID
#define M3508_MOTOR_SPEED_PID_KP 3.0f
#define M3508_MOTOR_SPEED_PID_KI 0.5f
#define M3508_MOTOR_SPEED_PID_KD 0.0f
#define M3508_MOTOR_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M3508_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

void motor_init(void);

#endif
