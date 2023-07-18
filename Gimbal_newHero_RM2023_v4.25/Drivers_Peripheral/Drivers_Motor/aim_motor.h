#ifndef AIM_MOTOR_H
#define AIM_MOTOR_H
#include "struct_typedef.h"
void Aim_Motor_Control(void);
void calculate_aim_motor_current_with_target_total_angle(void);
void aim_rotate_certain_angle(fp32 angle);
void aim_motor_init(void);
void calculate_aim_motor_current_with_target_total_speed(void);
#endif
