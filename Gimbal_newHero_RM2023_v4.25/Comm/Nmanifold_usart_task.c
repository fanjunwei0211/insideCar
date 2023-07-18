/**
  *****************************************************************************
  * @file       Nmanifold_usart_task.c/h
  * @brief      MANIFOLD data solve. MANIFOLD数据处理
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V2.0.0			Mar-3-2023			圣灵亮欣					1.Done
  *  V2.1.0			May-16-2023			圣灵亮欣					1.Add data transmission for dial switch
  @verbatim
  ==============================================================================
	
  ==============================================================================
  @endverbatim
  *****************************************************************************
*/
/*****************************************************************************************
			新C板——NUC串口通信协议（自瞄版本）C板底层使用步骤
①确保工程中含有bsp_usart.c和bsp_usart.h。
②确保NUC上的USB转串口模块与C板的USART1（即C板外壳上的UART2）连接正确。
③确保工程中的USART1及其接收、发送DMA均已在Cube中完成配置，且USART1的波特率须为115200。
④将该源文件导入到工程中，同时确保头文件在工程的包含目录中。
⑤在stm32f4xx_it.c的USART1_IRQHandler_1函数中调用USART1_IRQHandler_1函数。
⑥使用xTaskCreate或osThreadDef+osThreadCreate创建manifold_usart_task的FreeRTOS任务。
⑦检查串口收发数据流中的帧头、帧长、命令字和CRC校验位是否正常。
⑧在一切底层配置均正常的情况下即可在AutoAim_Data_Receive结构体内获取到自瞄相关数据。
*****************************************************************************************/
#include "Nmanifold_usart_task.h"
#include "bsp_usart.h"
#include "arm_math.h"
#include "Shoot_Task.h"
#include "referee.h"

extern UART_HandleTypeDef huart1;
extern ext_game_robot_state_t Game_Robot_State;

uint8_t Usart1_Buf[2][USART_RX_BUF_LENGHT], NUC_USART_RxBuf[USART_RX_BUF_LENGHT];
uint8_t Usart1_Dma_Txbuf[64] = {0};
Protocol_Head_Data Protocol_Head;
AutoAim_Data_Tx AutoAim_Data_Transmit;
AutoAim_Data_Rx AutoAim_Data_Receive;
Dial_Switch_Data Dial_Switch;

//uint8_t Autoaim_Mode = AUTOAIM_MODE_NORMAL;
//uint8_t Autoaim_Mode = AUTOAIM_MODE_SMALL_ENERGY;
uint8_t Autoaim_Mode =AUTOAIM_MODE_ANTI_TOP;

uint8_t Autoaim_Armor = AUTOAIM_ARMOR_AUTO;

void manifold_usart_task(void){
	static uint16_t t = 0;
	vTaskDelay(200);
	memset(NUC_USART_RxBuf, 0x00, USART_RX_BUF_LENGHT);								
	usart1_init(Usart1_Buf[0], Usart1_Buf[1], USART_RX_BUF_LENGHT);			
	vTaskDelay(200);
	while(1){
		if(++t > 3999) t = 0;
		
		//向NUC上位机发送自身云台姿态信息与射击参数，以获取自瞄所需信息
		if(t % 5 == 0){
			NUC_Usart_Tx(CMD_ID_AUTOAIM_DATA_TX);	
		}
		
		//向NUC上位机发送自瞄模式信息，指令NUC执行相应的自瞄算法
		if(t % 101 == 0){
			NUC_Usart_Tx(CMD_ID_WORKING_MODE);
		}	
		
		//向NUC上位机发送拨码开关数据信息，让NUC确定敌方队伍的颜色特征和敌方各个步兵的装甲板大小
		if(t % 2003 == 0){
			NUC_Usart_Tx(CMD_ID_DIAL_SWITCH);
		}
		
		vTaskDelay(1);
	}
}

void USART1_IRQHandler_1(void)
	{
	static uint16_t RX_Len_Now = 0;
	if(USART1->SR & UART_FLAG_IDLE)
		{
		__HAL_UART_CLEAR_PEFLAG(&huart1);
		__HAL_DMA_DISABLE(huart1.hdmarx);
		RX_Len_Now = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
		__HAL_DMA_SET_COUNTER(huart1.hdmarx, USART_RX_BUF_LENGHT);
			
		if((huart1.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
		{
			huart1.hdmarx->Instance->CR |= DMA_SxCR_CT;
			memcpy(NUC_USART_RxBuf, (char*)Usart1_Buf[0], RX_Len_Now);
		}
		else
		{
			huart1.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
			memcpy(NUC_USART_RxBuf, (char*)Usart1_Buf[1], RX_Len_Now);
		}
		__HAL_DMA_ENABLE(huart1.hdmarx);
		
		
		NUC_Data_Unpack();//数据解包
	}
}

uint8_t NUC_Data_Unpack(void)
{
	if(NUC_USART_RxBuf[0] != 0xAA)//我猜是标识符，标识符不是0xAA就返回1
		return 1;
	
	switch(NUC_USART_RxBuf[2])
	{
		case CMD_ID_AUTOAIM_DATA_RX:
			if(NUC_USART_RxBuf[1] != LENGTH_AUTOAIM_DATA_RX + 4 || CRC_Calculation(NUC_USART_RxBuf, LENGTH_AUTOAIM_DATA_RX + 3) != NUC_USART_RxBuf[3 + LENGTH_AUTOAIM_DATA_RX])
				return 1;
			memcpy(&AutoAim_Data_Receive, NUC_USART_RxBuf + 3, sizeof(AutoAim_Data_Rx));
		break;
		
		default:
			return 1;
	}
	return 0;
}
uint8_t  shijuefasong_or_not_watch=5;
void NUC_Usart_Tx(uint8_t cmdid){
	Working_Mode WMode;
	Protocol_Head.Header = 0xAA;
	Protocol_Head.Cmd_ID = cmdid;
	switch(cmdid){
		case CMD_ID_AUTOAIM_DATA_TX:
			Protocol_Head.Length = LENGTH_AUTOAIM_DATA_TX + 4;
			memcpy(Usart1_Dma_Txbuf, (uint8_t *)(&Protocol_Head), 3);
			AutoAim_Data_Transmit.Pitch = -INS_angle_deg[2];//视觉那边得要反一反
			AutoAim_Data_Transmit.Roll = INS_angle_deg[1];
			AutoAim_Data_Transmit.Yaw = INS_angle_deg[0];
			switch(Game_Robot_State.shooter_id1_42mm_speed_limit){
				case 10:		//弹速：10m/s
					AutoAim_Data_Transmit.Bullet_Speed = 10;
				break;
				case 16:		//弹速：16m/s
					AutoAim_Data_Transmit.Bullet_Speed = 16;
				break;
				default:
					AutoAim_Data_Transmit.Bullet_Speed = 0;
				break;
			}
			memcpy(Usart1_Dma_Txbuf + 3, (uint8_t *)(&AutoAim_Data_Transmit), LENGTH_AUTOAIM_DATA_TX);
			
			Usart1_Dma_Txbuf[LENGTH_AUTOAIM_DATA_TX + 3] = CRC_Calculation(Usart1_Dma_Txbuf, LENGTH_AUTOAIM_DATA_TX + 3);
			
			HAL_UART_Transmit_DMA(&huart1, Usart1_Dma_Txbuf, LENGTH_AUTOAIM_DATA_TX + 6-2);
		break;
		
		case CMD_ID_WORKING_MODE:
			Protocol_Head.Length = LENGTH_WORKING_MODE + 4;
			memcpy(Usart1_Dma_Txbuf,(uint8_t *)(&Protocol_Head), 3);
			WMode = Autoaim_Armor | Autoaim_Mode;
			memcpy(Usart1_Dma_Txbuf + 3, (uint8_t *)(&WMode), LENGTH_WORKING_MODE);
			
			Usart1_Dma_Txbuf[LENGTH_WORKING_MODE + 3] = CRC_Calculation(Usart1_Dma_Txbuf, LENGTH_WORKING_MODE + 3);
			
			HAL_UART_Transmit_DMA(&huart1, Usart1_Dma_Txbuf, LENGTH_WORKING_MODE + 6-2);
		break;
		
		case CMD_ID_DIAL_SWITCH:
			Protocol_Head.Length = LENGTH_DIAL_SWITCH + 4;
			memcpy(Usart1_Dma_Txbuf,(uint8_t *)(&Protocol_Head), 3);
			/*原来这个代码带拨码开关，测试没接，先注释掉*/
//			Dial_Switch.Enemy_Color					= HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_9) ? ENEMY_COLOR_BLUE				 	: ENEMY_COLOR_RED;
//			Dial_Switch.Infantry_Armor[0] 	= HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7) ? ENEMY_INFANTRY_ARMOR_BIG	: ENEMY_INFANTRY_ARMOR_SMALL;
//			Dial_Switch.Infantry_Armor[1] 	= HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0) ? ENEMY_INFANTRY_ARMOR_BIG 	: ENEMY_INFANTRY_ARMOR_SMALL;
//			Dial_Switch.Infantry_Armor[2] 	= HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_1) ? ENEMY_INFANTRY_ARMOR_BIG	: ENEMY_INFANTRY_ARMOR_SMALL;
//			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) ? GPIO_PIN_RESET : GPIO_PIN_SET);
				/*没接拨码开关的测试程序*/
			Dial_Switch.Enemy_Color					= ENEMY_COLOR_BLUE	;
			Dial_Switch.Infantry_Armor[0] 	= ENEMY_INFANTRY_ARMOR_SMALL;
			Dial_Switch.Infantry_Armor[1] 	=  ENEMY_INFANTRY_ARMOR_SMALL;
			Dial_Switch.Infantry_Armor[2] 	=  ENEMY_INFANTRY_ARMOR_SMALL;
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) ? GPIO_PIN_RESET : GPIO_PIN_SET);
		
			memcpy(Usart1_Dma_Txbuf + 3, (uint8_t *)(&Dial_Switch), LENGTH_DIAL_SWITCH);
			
			Usart1_Dma_Txbuf[LENGTH_DIAL_SWITCH + 3] = CRC_Calculation(Usart1_Dma_Txbuf, LENGTH_DIAL_SWITCH + 3);
			
			shijuefasong_or_not_watch=HAL_UART_Transmit_DMA(&huart1, Usart1_Dma_Txbuf, LENGTH_DIAL_SWITCH + 6-2);
		break;
		
		default:
			return;
	}
}

uint8_t CRC_Calculation(uint8_t *ptr, uint16_t len) {
	uint8_t crc = 0xff;
	while (len--) {
			crc = CRC08_Table[crc ^ *ptr++];
	}
	return crc;
}
