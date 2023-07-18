#include "can_receive.h"
#include "can.h"
#include "motor_can.h"
#include "pitch_motor.h"
#include "string.h"//����memcpy()����
#include "yaw_motor.h"
#include "Referee.h"
/**	[1]						canRX�ص�����
  * brief         	canÿ������һ֡���ݽ�������жϺ��жϻص����������жϺ������洦����յ������ݣ�ʵʱ���µ������
	* postscript			CAN2:Ħ���ֵ��ID=1��2��CAN1��yaw����ID=1 �����̵��ID=7						*/
extern motor_data_t fric_motor_data[2];
extern motor_data_t yaw_motor_data;
extern motor_data_t dial_motor_data[2];
float shoot_bullet_data=1.0f;
uint8_t bullet_data[4];//����������������С

extern ext_game_robot_state_t Game_Robot_State;
extern ext_power_heat_data_t  Power_Heat_Data;
extern motor_data_t 		aim_motor_data;	
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
		CAN_RxHeaderTypeDef rx_header;
		uint8_t rx_data[8];
	
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
	
	
		/**(1) ����CAN1������ **/
		if(hcan->Instance==CAN1)
		{
			
//			get_motor4310_data(&rx_header,rx_data,0);//�����ϰ��������ݣ�		
			
			switch(rx_header.StdId)
			{
				case (0x205):
				{
					get_motor6020_data(&yaw_motor_data,rx_data);//6020�ı�ʶ��������0x204+ID������0x205�ı�ʶ����Ӧ��6020ID��1
					break;
				}
				case 0x201://�²�����3508
				{
					get_motor3508_data(&dial_motor_data[1],rx_data);
					calculate_motor3508_total_angle(&dial_motor_data[1]);
					break;
				}
				case 0x202://�ϲ�����2006
				{
					get_motor3508_data(&dial_motor_data[0],rx_data);
					calculate_motor3508_total_angle(&dial_motor_data[0]);//���㲦���̵���������ܽǶ�break;	
					break;					
				}
				case 0x162:
				{
					memcpy(&bullet_data,&rx_data,sizeof(bullet_data));//���Խ��յ��ƺ�����A�����������
					memcpy(&shoot_bullet_data,&rx_data,sizeof(bullet_data));
					break;
				}
				case 0x152:
				{
					Power_Heat_Data.shooter_id1_42mm_cooling_heat   = (((uint16_t)rx_data[0]) << 8 | rx_data[1]);//�ĳɽ���ǹ��ʵʱ��������
					Game_Robot_State.shooter_id1_42mm_cooling_limit = (((uint16_t)rx_data[2]) << 8 | rx_data[3]);
					Game_Robot_State.shooter_id1_42mm_speed_limit   = (((uint16_t)rx_data[4]) << 8 | rx_data[5]);
					Game_Robot_State.mains_power_shooter_output     = rx_data[6];
					Game_Robot_State.mains_power_chassis_output     = rx_data[7];
					break;
					
				case 0x153:
				{
					Game_Robot_State.remain_HP   = (((uint16_t)rx_data[0]) << 8 | rx_data[1]);//����ʣ��Ѫ��
					Game_Robot_State.chassis_power_limit=(((uint16_t)rx_data[2]) << 8 | rx_data[3]);//���̹�������
					break;
				}
				
				default:break;
			}
			
		}
	}
		
		/**(2) ����CAN2������ **/
		if(hcan->Instance==CAN2)
		{
			get_motor4310_data(&rx_header,rx_data,0);//�����ϰ��������ݣ�
			switch(rx_header.StdId)
			{
				case 0x201:get_motor3508_data(&fric_motor_data[0],rx_data);break;
				case 0x202:get_motor3508_data(&fric_motor_data[1],rx_data);break;
				case 0x203://��׼��2006
				{
					get_motor3508_data(&aim_motor_data,rx_data);
					calculate_motor3508_total_angle(&aim_motor_data);//������׼������������ܽǶ�break;	
					break;					
				}
				default:break;
			}
			
		}
}
