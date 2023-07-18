#ifndef _SHOOT_TASK
#define _SHOOT_TASK
#include "struct_typedef.h"

void up_dial_reset(void);
void up_dial_rotate_certain_angle(fp32 angle);
void down_dial_rotate_until_locked_rotor(void);
void fric_rotate_at_certain_speed(int16_t fric_motor_speed);
void fric_dectct_speed_loss(void);
void fric_rotate_by_PWM(uint16_t fric_target_speed);
void down_dial_rotate_certain_angle(fp32 angle);
void up_dial_dial_locked_return(void);
void locked_rotor_detect(int16_t give_current,int16_t current_threshold,uint16_t locked_rotor_cnt_threshold,uint8_t *locked_rotor_flag,uint16_t *locked_rotor_cnt);
void up_dial_reset_and_Reset_Direction_locked_rotor_detect(void);
#endif
