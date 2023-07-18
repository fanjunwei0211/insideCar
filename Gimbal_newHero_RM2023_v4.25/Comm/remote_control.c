#include "remote_control.h"
#include "main.h"//���Ӳ���ʶUART_HandleTypeDef
#include "bsp_rc.h"

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

/**[1]						
  * brief						ң������ʼ������ʵ����������һ�´��ڽ��յ���ң�������ݴ浽�ڴ�������������
	* postscript		  ��ϸ�����ú���RC_init()�ڡ�bsp_rc.c��*/
static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];
void remote_control_init()
{
		RC_init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
}




/**[2]						
  * brief						����3�жϴ�����
	* postscript		  ΪʲôҪ�жϻ��������ж����ڵĻ�������Memory0����Memory1,�����0����1�е����ݣ������1����0�е���Ϣ������ʱ����Ӱ����һ���������������ݵĽ��գ�*/
RC_ctrl_t rc_ctrl;
void USART3_IRQHandler(void)
{
		/*��1��	����ǽ����ж�*/
    if(huart3.Instance->SR & UART_FLAG_RXNE)//����ǽ����ж�			SR��Status register,״̬�Ĵ�����	RXNE��RX Not Empty����������յ�����
    {
        __HAL_UART_CLEAR_PEFLAG(&huart3);//���мĴ����жϱ�־λ�Ĵ���PEFLAG��PE flag��PE������ԭ��ͼ�����ŵ����֣������жϺ���
    }
		
		/*��2��	����ǿ����ж�*/
    else if(USART3->SR & UART_FLAG_IDLE)//����ǿ����ж�		idle�����е�?��		���ڽ�����һ֡���ݣ�һ֡�����ɶ���ֽ���ɣ��Լ�����һ�ν��ն������ݣ���IDLEλ����1��
    {
        static uint16_t this_time_rx_len = 0;
				
				/*��2.1��	�����жϱ�־λ*/
        __HAL_UART_CLEAR_PEFLAG(&huart3);

				/*��2.2���жϽ��н��յĻ�������1�Ż���������2�Ż��������������ݴ浽��Ӧ������*/
			
					/*��2.2.1�� �����1�Ż�����*/
        if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)	//ͨ�����CTλ�жϣ�ä�����CR�Ĵ��������CTλ���������趨�þͲ���ģ��������Ż��������л������
        {
            /*��2.2.1.1��	ʧЧDMA*/
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

						/*��2.2.1.2��	��ȡ����DMA���յ������ݳ���,���� = �趨���� - ʣ�೤��*/
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;//NDTR��number of data items to transfer register��Ҫ�������������Ŀ��

						/*��2.2.1.3��	�����趨���ݳ���,�Ա��´μ�����*/
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

						/*��2.2.1.4��	�޸�CR�Ĵ�����CTλ��CTλ��1���л���������2*/
            hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;//��������������������������Զ��л�����������������һ���������ĳ�����һ֡���ݳ��ȵ��������϶������������Ҫ�ֶ��л���������
            
            /*��2.2.1.5��	��������ʹ��DMA*/
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

						/*��2.2.1.6��	ң�������ݽ���*/
            if(this_time_rx_len == RC_FRAME_LENGTH)//�ж��Ƿ���һ֡���ݣ�18 �ֽڣ�������ȣ�����������ú��� sbus_to_rc ����ң�������ݵĽ��롣
            {
                sbus_to_rc(sbus_rx_buf[0], &rc_ctrl);
            }
        }
				
					/*��2.2.2�� �����2�Ż�����*/
        else
        {
            /*��2.2.2.1�� 	ʧЧDMA*/
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

						/*��2.2.2.2��	��ȡ����DMA���յ������ݳ���,���� = �趨���� - ʣ�೤��*/
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            /*��2.2.2.3��	�����趨���ݳ���,�Ա��´μ�����*/
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            /*��2.2.2.4��	�޸�CR�Ĵ�����CTλ��CTλ��0���л���������1*/
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
            
            /*��2.2.2.5��	��������ʹ��DMA*/
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

						/*��2.2.2.6��	ң�������ݽ���*/
            if(this_time_rx_len == RC_FRAME_LENGTH)//�ж��Ƿ���һ֡���ݣ�18 �ֽڣ�������ȣ�����������ú��� sbus_to_rc ����ң�������ݵĽ��롣
            {
                sbus_to_rc(sbus_rx_buf[1], &rc_ctrl);
            }
        }
    }
}

/**[3]						
  * brief						ң�������ݽ���
	*	param[in]				sbus_buf:���ң����ԭʼ���ݵ��ڴ������ң�����������ݵ����ڣ�����ͨ��DMA�����ݷ��͵��ڴ滺����
	*	param[out]			rc_ctrl:ң�����ṹ�����������������ݴ��������ṹ������
	* postscript		  */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }
		/*��1��	ҡ������*/
    rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
    rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 2
                         (sbus_buf[4] << 10)) &0x07ff;
    rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
		
		/*��2��	��������*/
    rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                  //!< Switch left
    rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                       //!< Switch right
		
		/*��3��	�������*/
    rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< Mouse X axis
    rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis
    rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis
    rc_ctrl->mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press ?
    rc_ctrl->mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press ?
		
		/*��4��	�������ݣ�*/
    rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value
		
		/*��5��	�ǲ���ң�����������ݣ�*/
    rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 //NULL

		/*��6��	��һ��ң�������ݷ�Χ*/
    rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;//��1024��ң��������ҡ����ֵ��-660��+660
    rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
}
