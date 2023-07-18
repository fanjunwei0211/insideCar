#include "bsp_rc.h"
#include "main.h"


/**[1]						
  * brief						����3����ң�������ݣ�����CPUֱ��ͨ��DMA���䵽�ڴ��������
	* param[out]			rx1_buf:�ڴ滺����1
	* param[out]			rx2_buf:�ڴ滺����2
	*	param[in]				���ճ��ȣ���
	* postscript		  Ϊʲô��˫��������˫���������ڽ�����һ֡���ݺ�DMA �������ʱ���л�Ŀ��洢������������ڴ�����һ֡����ʱ����ʱ������µ�һ֡���ݾͲ��Ḳ��ԭ�������ݡ� */
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;
void RC_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
		/**(1)	���ô���**/
	
    /*(1.1)	ʹ�ܴ���DMA����*/
    SET_BIT(huart3.Instance->CR3, USART_CR3_DMAR);
	
		/*(1.2)	ʹ�ܴ��ڿ����ж�*/
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);

	
	
    /**(2) ����DMA**/
	
    /*(2.1)	ʧЧ����DMA����DMA��һЩ����*/
    __HAL_DMA_DISABLE(&hdma_usart3_rx);							//ʧЧ����3��DMA����ΪDMA�����б�������ʧЧ�Ĳ���
    while(hdma_usart3_rx.Instance->CR & DMA_SxCR_EN)//����DMAʹ�ܱ�־λ��û����0��û�������ʧЧ
    {
        __HAL_DMA_DISABLE(&hdma_usart3_rx);
    }
		
		/*��2.2�� ����DMA ������˭������3��������������ͨ��DMA�����ڴ���������ڴ滺��������Ҫ�������ݳ���*/
		
			/*��2.2.1��	����DMA������˭*/
    hdma_usart3_rx.Instance->PAR = (uint32_t) & (USART3->DR);//PAR��peripheral address register�������ַ�Ĵ���������Ŵ���3�����ݼĴ���DR�ĵ�ַ
    
			/*��2.2.2��	����DMA����������ͨ��DMA�����ڴ���������ڴ滺������*/
    hdma_usart3_rx.Instance->M0AR = (uint32_t)(rx1_buf);//MOAR��memory 0 address register���ڴ滺����1��ַ�Ĵ��������ڴ�����ݵĵط�Ҳ���������Լ�����ı����������ʾ���贮��3���յ�������ͨ��DMA���䵽�ڴ棬�ڴ�ѽ��յ������ݴ�ŵ�rx1_buf��������Լ�����ı�������
    hdma_usart3_rx.Instance->M1AR = (uint32_t)(rx2_buf);//M1AR��memory 1 address register���ڴ滺����2��ַ�Ĵ�����
    
			/*��2.2.3��	����DMAҪ�������ݳ���*/
    hdma_usart3_rx.Instance->NDTR = dma_buf_num;//NDTR��number of data items to transfer register��Ҫ�������������Ŀ��
    
			/*��2.2.4��	ʹ��DMA˫������*/
    SET_BIT(hdma_usart3_rx.Instance->CR, DMA_SxCR_DBM);//CR��configuration register�����üĴ�������Ҫ��˫��������Ҫ��ʹ��һ��

    
    /*(2.3)	������ɺ���ʹ�ܴ���DMA*/
    __HAL_DMA_ENABLE(&hdma_usart3_rx);

}

