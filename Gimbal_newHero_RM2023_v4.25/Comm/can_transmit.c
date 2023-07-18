#include "can_transmit.h"
#include "can.h"
#include "string.h"

/**	[1]				
  * brief       向底盘发送遥控器四个摇杆的数据  
  * postscript				*/
void send_chassis_target_speed_by_CAN1(uint8_t ChassisMode,int16_t chassis_target_forward_speed_x,int16_t chassis_target_traverse_speed_y,int16_t chassis_target_rotation_speed_z)
{
	CAN_TxHeaderTypeDef remote_control_sending_tx_message;
	uint8_t remote_control_sending_can_send_data[8];
	uint32_t send_mail_box;
	
	remote_control_sending_tx_message.StdId=0x150;
	remote_control_sending_tx_message.RTR=CAN_RTR_DATA;
	remote_control_sending_tx_message.IDE=CAN_ID_STD;
	remote_control_sending_tx_message.DLC=0x07;
	
	remote_control_sending_can_send_data[0]=ChassisMode;
	remote_control_sending_can_send_data[1]=chassis_target_forward_speed_x >> 8;//int16_t数据的高八位
	remote_control_sending_can_send_data[2]=chassis_target_forward_speed_x & 0xff;//int16_t数据的低八位
	remote_control_sending_can_send_data[3]=chassis_target_traverse_speed_y >> 8;
	remote_control_sending_can_send_data[4]=chassis_target_traverse_speed_y & 0xff;
	remote_control_sending_can_send_data[5]=chassis_target_rotation_speed_z >> 8;
	remote_control_sending_can_send_data[6]=chassis_target_rotation_speed_z & 0xff;

	HAL_CAN_AddTxMessage(&hcan1,&remote_control_sending_tx_message,remote_control_sending_can_send_data,&send_mail_box);
}

/**	[2]				
  * brief       向底盘发送遥控器四个摇杆的数据  
  * postscript				*/

void send_UI_data_for_Chassis_by_CAN1(uint8_t Autoaim_UI_flag,uint16_t xiaotuoluo_UI_flag,uint16_t fric_UI_flag,float INS_roll_for_UI,uint8_t down_dial_on_or_off_flag)
{
	CAN_TxHeaderTypeDef UI_data_sending_tx_message;
	uint8_t UI_data_sending_can_send_data[8];
	uint32_t send_mail_box;
	
	uint8_t data_sending_can_send_data[4];
	
	UI_data_sending_tx_message.StdId=0x151;
	UI_data_sending_tx_message.RTR=CAN_RTR_DATA;
	UI_data_sending_tx_message.IDE=CAN_ID_STD;
	UI_data_sending_tx_message.DLC=0x08;
	
	
	memcpy(UI_data_sending_can_send_data,&INS_roll_for_UI,sizeof(data_sending_can_send_data));
	UI_data_sending_can_send_data[4]=Autoaim_UI_flag;
	UI_data_sending_can_send_data[5]=xiaotuoluo_UI_flag;//小陀螺标志位
	UI_data_sending_can_send_data[6]=fric_UI_flag;//摩擦轮标志位
	UI_data_sending_can_send_data[7]=down_dial_on_or_off_flag;

	HAL_CAN_AddTxMessage(&hcan1,& UI_data_sending_tx_message,UI_data_sending_can_send_data,&send_mail_box);
}

/**	[3]				
  * brief         
  * postscript				*/

void send_UI_pitch_data_for_Chassis_by_CAN1(float INS_pitch_for_UI)
{
	CAN_TxHeaderTypeDef UI_data_sending_tx_message;
	uint8_t UI_data_sending_can_send_data[8];
	uint32_t send_mail_box;
	
	uint8_t data_sending_can_send_data[4];
	
	UI_data_sending_tx_message.StdId=0x152;
	UI_data_sending_tx_message.RTR=CAN_RTR_DATA;
	UI_data_sending_tx_message.IDE=CAN_ID_STD;
	UI_data_sending_tx_message.DLC=0x04;
	
	memcpy(UI_data_sending_can_send_data,&INS_pitch_for_UI,sizeof(data_sending_can_send_data));

	HAL_CAN_AddTxMessage(&hcan1,& UI_data_sending_tx_message,UI_data_sending_can_send_data,&send_mail_box);
}


/**	[3]							
  * brief         	发一个float型数据
	* postscript									*/
void send_one_float_data_through_CAN2(float one_float_data)
{
	CAN_TxHeaderTypeDef data_sending_tx_message;
	uint8_t data_sending_can_send_data[4];
	uint32_t send_mail_box;
	
	data_sending_tx_message.StdId=0x162;
	data_sending_tx_message.RTR=CAN_RTR_DATA;
	data_sending_tx_message.IDE=CAN_ID_STD;
	data_sending_tx_message.DLC=0x04;
	
	memcpy(data_sending_can_send_data,&one_float_data,sizeof(data_sending_can_send_data));
	HAL_CAN_AddTxMessage(&hcan2,&data_sending_tx_message,data_sending_can_send_data,&send_mail_box);
}
