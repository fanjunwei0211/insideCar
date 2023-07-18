#ifndef _DIAL_MOTOR_H
#define _DIAL_MOTOR_H
#include "struct_typedef.h"

void dial_motor_init(void);
void calculate_up_dial_motor_current_with_target_speed(void);
void calculate_down_dial_motor_current_with_target_speed(void);
void calculate_up_dial_motor_current_with_target_total_angle(void);
void calculate_down_dial_motor_current_with_target_total_angle(void);

#endif
