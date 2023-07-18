#include "can_receive.h"
#include "can.h"
#include "motor_can.h"
#include "pitch_motor.h"
#include "string.h"//调用memcpy()函数
#include "yaw_motor.h"
#include "Referee.h"
/**	[1]						canRX回调函数
  * brief         	can每接收完一帧数据进入接收中断和中断回调函数，在中断函数里面处理接收到的数据，实时更新电机数据
	* postscript			CAN2:摩擦轮电机ID=1，2；CAN1：yaw轴电机ID=1 拨弹盘电机ID=7						*/
extern motor_data_t fric_motor_data[2];
extern motor_data_t yaw_motor_data;
extern motor_data_t dial_motor_data[2];
float shoot_bullet_data=1.0f;
uint8_t bullet_data[4];//单纯用它算个数组大小

extern ext_game_robot_state_t Game_Robot_State;
extern ext_power_heat_data_t  Power_Heat_Data;
extern motor_data_t 		aim_motor_data;	
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
		CAN_RxHeaderTypeDef rx_header;
		uint8_t rx_data[8];
	
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
	
	
		/**(1) 接收CAN1的数据 **/
		if(hcan->Instance==CAN1)
		{
			
//			get_motor4310_data(&rx_header,rx_data,0);//接收老板电机的数据，		
			
			switch(rx_header.StdId)
			{
				case (0x205):
				{
					get_motor6020_data(&yaw_motor_data,rx_data);//6020的标识符规则是0x204+ID，所以0x205的标识符对应的6020ID是1
					break;
				}
				case 0x201://下拨弹盘3508
				{
					get_motor3508_data(&dial_motor_data[1],rx_data);
					calculate_motor3508_total_angle(&dial_motor_data[1]);
					break;
				}
				case 0x202://上拨弹盘2006
				{
					get_motor3508_data(&dial_motor_data[0],rx_data);
					calculate_motor3508_total_angle(&dial_motor_data[0]);//计算拨弹盘电机编码器总角度break;	
					break;					
				}
				case 0x162:
				{
					memcpy(&bullet_data,&rx_data,sizeof(bullet_data));//可以接收但似乎底盘A板读不到弹速
					memcpy(&shoot_bullet_data,&rx_data,sizeof(bullet_data));
					break;
				}
				case 0x152:
				{
					Power_Heat_Data.shooter_id1_42mm_cooling_heat   = (((uint16_t)rx_data[0]) << 8 | rx_data[1]);//改成接收枪口实时热量数据
					Game_Robot_State.shooter_id1_42mm_cooling_limit = (((uint16_t)rx_data[2]) << 8 | rx_data[3]);
					Game_Robot_State.shooter_id1_42mm_speed_limit   = (((uint16_t)rx_data[4]) << 8 | rx_data[5]);
					Game_Robot_State.mains_power_shooter_output     = rx_data[6];
					Game_Robot_State.mains_power_chassis_output     = rx_data[7];
					break;
					
				case 0x153:
				{
					Game_Robot_State.remain_HP   = (((uint16_t)rx_data[0]) << 8 | rx_data[1]);//接收剩余血量
					Game_Robot_State.chassis_power_limit=(((uint16_t)rx_data[2]) << 8 | rx_data[3]);//底盘功率限制
					break;
				}
				
				default:break;
			}
			
		}
	}
		
		/**(2) 接收CAN2的数据 **/
		if(hcan->Instance==CAN2)
		{
			get_motor4310_data(&rx_header,rx_data,0);//接收老板电机的数据，
			switch(rx_header.StdId)
			{
				case 0x201:get_motor3508_data(&fric_motor_data[0],rx_data);break;
				case 0x202:get_motor3508_data(&fric_motor_data[1],rx_data);break;
				case 0x203://瞄准镜2006
				{
					get_motor3508_data(&aim_motor_data,rx_data);
					calculate_motor3508_total_angle(&aim_motor_data);//计算瞄准镜电机编码器总角度break;	
					break;					
				}
				default:break;
			}
			
		}
}
