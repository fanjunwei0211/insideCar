#ifndef _YAW_MOTOR
#define	_YAW_MOTOR
#include "struct_typedef.h"

void yaw_motor_init(void);
void calculate_yaw_motor_current_with_target_INS_speed(void);
void calculate_yaw_motor_current_with_target_INS_angle(void);
void Yaw_Motor_Control(void);
void calculate_yaw_motor_current_with_autoaim_target_INS_speed(void);
void calculate_yaw_motor_current_with_autoaim_target_INS_angle(void);
void calculate_yaw_motor_current_with_target_encoder_speed(void);
void calculate_yaw_motor_current_with_target_encoder_angle(void);
void Yaw_Motor_Control_in_Autoaim_Mode(void);
void Yaw_Motor_Control_in_RC_Mode(void);
void Yaw_Motor_Control_in_Gimbal_Follow_Chassis_Mode(void);
void calculate_yaw_motor_current_with_target_key_INS_speed(void);
#endif
