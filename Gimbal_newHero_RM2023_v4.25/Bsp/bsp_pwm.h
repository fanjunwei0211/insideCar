#ifndef BSP_PWM_H
#define BSP_PWM_H
#include "struct_typedef.h"
#include "tim.h"//�����������ʶhtim������

void PWM_init(TIM_HandleTypeDef *htim, uint32_t Channel);
void imu_pwm_init(void);
void imu_pwm_set(uint16_t pwm);
#endif
