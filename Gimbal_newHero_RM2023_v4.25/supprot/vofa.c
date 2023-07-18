#include "vofa.h"
#include "usart.h"

Vofa_data2 vofa_data2={.tail={0x00,0x00,0x80,0x7f}};//�ṹ�������ʼ��
Vofa_data4 vofa_data4={.tail={0x00,0x00,0x80,0x7f}};//�ṹ�������ʼ��


/** [1]						��VOFA��λ������������
  * brief         
	* postscript		*/
void send_data_to_vofa2(float data1, float data2)
{
		vofa_data2.data[0]=data1;
		vofa_data2.data[1]=data2;
		HAL_UART_Transmit_DMA(&huart6,(uint8_t *)&vofa_data2,sizeof(vofa_data2));
}


/** [1]						��VOFA��λ�����ĸ�����
  * brief         
	* postscript		*/
void send_data_to_vofa4(float data1, float data2, float data3, float data4)
{
		vofa_data4.data[0]=data1;
		vofa_data4.data[1]=data2;
		vofa_data4.data[2]=data3;
		vofa_data4.data[3]=data4;
		HAL_UART_Transmit_DMA(&huart6,(uint8_t *)&vofa_data4,sizeof(vofa_data4));
}
