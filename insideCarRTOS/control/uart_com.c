#include "uart_com.h"
#include "string.h"

uint8_t uart6_data;
uint8_t uart6_rxbuf[4];  //0x55帧头
uint8_t uart6_txbuf[4] = {0};  //0x55帧头  检测完否-停下是否  行进距离  
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
	
	ucRxBuffer[ucRxCnt++]=data;	//将收到的数据存入缓冲区中
	if (ucRxBuffer[0]!=0x55) //数据头不对，则重新开始寻找0x55数据头
	{
		ucRxCnt=0;
		return;
	}
	if (ucRxCnt<4) {return;}//数据不满11个，则返回
	else
	{
		memcpy(&uart6_rxbuf,ucRxBuffer,5);
		ucRxCnt=0;//清空缓存区
	}
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart==&huart6)
	{
//		Uart6_Receive_Data(uart6_data);//处理数据
		if(uart6_data == 0xaa)
			start_flag = 1;
		HAL_UART_Receive_IT(&huart6,&uart6_data,1);	
	}
}

