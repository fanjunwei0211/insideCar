#include "bsp_rc.h"
#include "main.h"


/**[1]						
  * brief						串口3接收遥控器数据，不经CPU直接通过DMA传输到内存数组变量
	* param[out]			rx1_buf:内存缓冲区1
	* param[out]			rx2_buf:内存缓冲区2
	*	param[in]				接收长度？？
	* postscript		  为什么用双缓冲区？双缓冲区会在接收完一帧数据后（DMA 传输结束时）切换目标存储区。这样如果在处理上一帧数据时，这时候接收新的一帧数据就不会覆盖原来的数据。 */
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;
void RC_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
		/**(1)	配置串口**/
	
    /*(1.1)	使能串口DMA接收*/
    SET_BIT(huart3.Instance->CR3, USART_CR3_DMAR);
	
		/*(1.2)	使能串口空闲中断*/
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);

	
	
    /**(2) 配置DMA**/
	
    /*(2.1)	失效串口DMA，对DMA做一些配置*/
    __HAL_DMA_DISABLE(&hdma_usart3_rx);							//失效串口3的DMA，因为DMA好像有保护，不失效改不了
    while(hdma_usart3_rx.Instance->CR & DMA_SxCR_EN)//看看DMA使能标志位有没有置0，没置零继续失效
    {
        __HAL_DMA_DISABLE(&hdma_usart3_rx);
    }
		
		/*（2.2） 告诉DMA 外设是谁（串口3）、外设存的数据通过DMA传到内存哪里（两个内存缓冲区）、要传的数据长度*/
		
			/*（2.2.1）	告诉DMA外设是谁*/
    hdma_usart3_rx.Instance->PAR = (uint32_t) & (USART3->DR);//PAR（peripheral address register，外设地址寄存器），存放串口3的数据寄存器DR的地址
    
			/*（2.2.2）	告诉DMA外设存的数据通过DMA传到内存哪里（两个内存缓冲区）*/
    hdma_usart3_rx.Instance->M0AR = (uint32_t)(rx1_buf);//MOAR（memory 0 address register，内存缓冲区1地址寄存器），内存存数据的地方也就是我们自己定义的变量，这里表示外设串口3接收到的数据通过DMA传输到内存，内存把接收到的数据存放到rx1_buf这个我们自己定义的变量里面
    hdma_usart3_rx.Instance->M1AR = (uint32_t)(rx2_buf);//M1AR（memory 1 address register，内存缓冲区2地址寄存器）
    
			/*（2.2.3）	告诉DMA要传的数据长度*/
    hdma_usart3_rx.Instance->NDTR = dma_buf_num;//NDTR（number of data items to transfer register，要传输的数据项数目）
    
			/*（2.2.4）	使能DMA双缓冲区*/
    SET_BIT(hdma_usart3_rx.Instance->CR, DMA_SxCR_DBM);//CR（configuration register，配置寄存器），要用双缓冲区还要再使能一下

    
    /*(2.3)	配置完成后，再使能串口DMA*/
    __HAL_DMA_ENABLE(&hdma_usart3_rx);

}

