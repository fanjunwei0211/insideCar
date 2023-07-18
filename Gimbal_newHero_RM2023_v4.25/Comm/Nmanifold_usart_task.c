/**
  *****************************************************************************
  * @file       Nmanifold_usart_task.c/h
  * @brief      MANIFOLD data solve. MANIFOLD���ݴ���
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V2.0.0			Mar-3-2023			ʥ������					1.Done
  *  V2.1.0			May-16-2023			ʥ������					1.Add data transmission for dial switch
  @verbatim
  ==============================================================================
	
  ==============================================================================
  @endverbatim
  *****************************************************************************
*/
/*****************************************************************************************
			��C�塪��NUC����ͨ��Э�飨����汾��C��ײ�ʹ�ò���
��ȷ�������к���bsp_usart.c��bsp_usart.h��
��ȷ��NUC�ϵ�USBת����ģ����C���USART1����C������ϵ�UART2��������ȷ��
��ȷ�������е�USART1������ա�����DMA������Cube��������ã���USART1�Ĳ�������Ϊ115200��
�ܽ���Դ�ļ����뵽�����У�ͬʱȷ��ͷ�ļ��ڹ��̵İ���Ŀ¼�С�
����stm32f4xx_it.c��USART1_IRQHandler_1�����е���USART1_IRQHandler_1������
��ʹ��xTaskCreate��osThreadDef+osThreadCreate����manifold_usart_task��FreeRTOS����
�߼�鴮���շ��������е�֡ͷ��֡���������ֺ�CRCУ��λ�Ƿ�������
����һ�еײ����þ�����������¼�����AutoAim_Data_Receive�ṹ���ڻ�ȡ������������ݡ�
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
		
		//��NUC��λ������������̨��̬��Ϣ������������Ի�ȡ����������Ϣ
		if(t % 5 == 0){
			NUC_Usart_Tx(CMD_ID_AUTOAIM_DATA_TX);	
		}
		
		//��NUC��λ����������ģʽ��Ϣ��ָ��NUCִ����Ӧ�������㷨
		if(t % 101 == 0){
			NUC_Usart_Tx(CMD_ID_WORKING_MODE);
		}	
		
		//��NUC��λ�����Ͳ��뿪��������Ϣ����NUCȷ���з��������ɫ�����͵з�����������װ�װ��С
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
		
		
		NUC_Data_Unpack();//���ݽ��
	}
}

uint8_t NUC_Data_Unpack(void)
{
	if(NUC_USART_RxBuf[0] != 0xAA)//�Ҳ��Ǳ�ʶ������ʶ������0xAA�ͷ���1
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
			AutoAim_Data_Transmit.Pitch = -INS_angle_deg[2];//�Ӿ��Ǳߵ�Ҫ��һ��
			AutoAim_Data_Transmit.Roll = INS_angle_deg[1];
			AutoAim_Data_Transmit.Yaw = INS_angle_deg[0];
			switch(Game_Robot_State.shooter_id1_42mm_speed_limit){
				case 10:		//���٣�10m/s
					AutoAim_Data_Transmit.Bullet_Speed = 10;
				break;
				case 16:		//���٣�16m/s
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
			/*ԭ�������������뿪�أ�����û�ӣ���ע�͵�*/
//			Dial_Switch.Enemy_Color					= HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_9) ? ENEMY_COLOR_BLUE				 	: ENEMY_COLOR_RED;
//			Dial_Switch.Infantry_Armor[0] 	= HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7) ? ENEMY_INFANTRY_ARMOR_BIG	: ENEMY_INFANTRY_ARMOR_SMALL;
//			Dial_Switch.Infantry_Armor[1] 	= HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0) ? ENEMY_INFANTRY_ARMOR_BIG 	: ENEMY_INFANTRY_ARMOR_SMALL;
//			Dial_Switch.Infantry_Armor[2] 	= HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_1) ? ENEMY_INFANTRY_ARMOR_BIG	: ENEMY_INFANTRY_ARMOR_SMALL;
//			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) ? GPIO_PIN_RESET : GPIO_PIN_SET);
				/*û�Ӳ��뿪�صĲ��Գ���*/
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
