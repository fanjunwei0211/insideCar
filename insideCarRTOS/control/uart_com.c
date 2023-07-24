#include "uart_com.h"
#include "string.h"

uint8_t uart6_data;
uint8_t uart6_rxbuf[4];  //0x55֡ͷ
uint8_t uart6_txbuf[4] = {0};  //0x55֡ͷ  ������-ͣ���Ƿ�  �н�����  
volatile char start_flag;


void usart_start_it()
{
	HAL_UART_Receive_IT(&huart6, &uart6_data, 1);
}

void Uart6_Send_Data()
{
	uart6_txbuf[0] = 0x55;
	HAL_UART_Transmit_DMA(&huart6,uart6_txbuf,sizeof(uart6_txbuf));
}




void Uart6_Receive_Data(uint8_t data)
{
	static unsigned char ucRxBuffer[50];
	static unsigned char ucRxCnt = 0;	
	
	ucRxBuffer[ucRxCnt++]=data;	//���յ������ݴ��뻺������
	if (ucRxBuffer[0]!=0x55) //����ͷ���ԣ������¿�ʼѰ��0x55����ͷ
	{
		ucRxCnt=0;
		return;
	}
	if (ucRxCnt<4) {return;}//���ݲ���11�����򷵻�
	else
	{
		memcpy(&uart6_rxbuf,ucRxBuffer,5);
		ucRxCnt=0;//��ջ�����
	}
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart==&huart6)
	{
//		Uart6_Receive_Data(uart6_data);//��������
		if(uart6_data == 0xaa)
			start_flag = 1;
		HAL_UART_Receive_IT(&huart6,&uart6_data,1);	
	}
}

