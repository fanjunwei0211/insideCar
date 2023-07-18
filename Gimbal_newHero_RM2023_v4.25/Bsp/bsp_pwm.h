#ifndef BSP_PWM_H
#define BSP_PWM_H
#include "struct_typedef.h"
#include "tim.h"//不加这个不认识htim的类型

void PWM_init(TIM_HandleTypeDef *htim, uint32_t Channel);
void imu_pwm_init(void);
void imu_pwm_set(uint16_t pwm);
#endif
