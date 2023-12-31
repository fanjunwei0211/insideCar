#ifndef CAN_TRANSMIT_H
#define CAN_TRANSMIT_H
#include "struct_typedef.h"

void send_chassis_target_speed_by_CAN1(uint8_t ChassisMode,int16_t chassis_target_forward_speed_x,int16_t chassis_target_traverse_speed_y,int16_t chassis_target_rotation_speed_z);
void send_UI_data_for_Chassis_by_CAN1(uint8_t Autoaim_UI_flag,uint16_t xiaotuoluo_UI_flag,uint16_t fric_UI_flag,float INS_roll_for_UI,uint8_t down_dial_on_or_off_flag);
void send_UI_pitch_data_for_Chassis_by_CAN1(float INS_pitch_for_UI);
#endif
