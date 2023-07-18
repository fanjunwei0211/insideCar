#include "bsp_spi.h"
#include "main.h"

/**[1]						
  * brief					配置SPI的DMA
	* param[in]				要发送的数据存储的内存地址
	* param[in]				接收的数据存储的内存地址
    * param[in]             传输数据的长度
	* postscript		    由于SPI是MISO和MOSI同时进行传输数据，所以需要同时开启SPI的RX和TX*/
extern SPI_HandleTypeDef hspi1;
extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;
void SPI1_DMA_init(uint32_t tx_buf,uint32_t rx_buf,uint16_t num)
{
    /**(1)   配置SPI**/
    SET_BIT(hspi1.Instance->CR2,SPI_CR2_TXDMAEN);//使能SPI发送DMA？
    SET_BIT(hspi1.Instance->CR2,SPI_CR2_RXDMAEN);//使能SPI接收DMA？
    __HAL_SPI_ENABLE(&hspi1);//使能SPI


    /**(2)   配置SPI接收rx的DMA**/
    /*(2.1) 失效DMA*/
    __HAL_DMA_DISABLE(&hdma_spi1_rx);
    
    while(hdma_spi1_rx.Instance->CR & DMA_SxCR_EN)//如果失效成功继续失效
    {
        __HAL_DMA_DISABLE(&hdma_spi1_rx);
    }
    /*(2.2) 传输完成标志位置0*/
    __HAL_DMA_CLEAR_FLAG(&hdma_spi1_rx,DMA_LISR_TCIF2);/*DMA_LISR(DMA low interrupt status register,DMA低中断状态寄存器)
                                                        TCIFx(Stream x transfer complete interrupt flag,数据流x传输完成中断标志),此位由硬件置1，软件置0。
                                                        1:表示数据流x上发生传输完成事件;0:无传输完成事件*/
    /*(2.3) 告诉DMA 外设是谁（SPI）、外设存的数据通过DMA传到内存哪里(rx_buf)、要传的数据长度(num)*/
    hdma_spi1_rx.Instance->PAR = (uint32_t) & (SPI1->DR);
    hdma_spi1_rx.Instance->M0AR = (uint32_t)(rx_buf);
    __HAL_DMA_SET_COUNTER(&hdma_spi1_rx, num);
    /*(2.4) 使能DMA传输完成中断*/
    __HAL_DMA_ENABLE_IT(&hdma_spi1_rx, DMA_IT_TC);//TC(transfer complete),实际上是DMA_SxCR寄存器的TXIE位 置1


    /**(3)  配置SPI发送tx的DMA**/
    /*(3.1) 失效DMA*/
    __HAL_DMA_DISABLE(&hdma_spi1_tx);
    
    while(hdma_spi1_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_spi1_tx);
    }
    /*(3.2) 传输完成标志位置0*/
    __HAL_DMA_CLEAR_FLAG(&hdma_spi1_tx, DMA_LISR_TCIF3);
    /*(3.3) 告诉DMA 外设是谁（SPI）、内存通过DMA要发送的数据在哪里(tx_buf)、要传的数据长度(num)*/
    hdma_spi1_tx.Instance->PAR = (uint32_t) & (SPI1->DR);
    hdma_spi1_tx.Instance->M0AR = (uint32_t)(tx_buf);
    __HAL_DMA_SET_COUNTER(&hdma_spi1_tx, num);
}
/**[2]						
  * brief					使能SPI的DMA
	* param[in]				要发送的数据存储的内存地址
	* param[in]				接收的数据存储的内存地址
    * param[in]             传输数据的长度
	* postscript		    使用 SPI 通信时，需要保证同一时刻只有一个设备在通信，即同时只能获取陀螺仪的数据或者加速度计的数据，故而需要对SPI的通信进行调度。\
                            所以在SPI1_DMA_init()函数中，失能DMA之后没有像RC_init()那样再使能，\
                            而是单独把使能函数单独再搞成一个函数，在某个设备要用SPI传输时使能，用完再失能给下一个设备用*/
  void SPI1_DMA_enable(uint32_t tx_buf,uint32_t rx_buf,uint16_t ndtr)                          
{
    /**(1)   失效DMA**/
    /*(1.1) 失效SPI接收的DMA*/
    __HAL_DMA_DISABLE(&hdma_spi1_rx);
    while(hdma_spi1_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_spi1_rx);
    }
    /*(1.2) 失效SPI发送的DMA*/
    __HAL_DMA_DISABLE(&hdma_spi1_tx);
    while(hdma_spi1_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_spi1_tx);
    }

    /**(2)  清除标志位**/
    /*(2.1) 清除rx的标志位*/
    __HAL_DMA_CLEAR_FLAG (hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx));
    __HAL_DMA_CLEAR_FLAG (hspi1.hdmarx, __HAL_DMA_GET_HT_FLAG_INDEX(hspi1.hdmarx));
    __HAL_DMA_CLEAR_FLAG (hspi1.hdmarx, __HAL_DMA_GET_TE_FLAG_INDEX(hspi1.hdmarx));
    __HAL_DMA_CLEAR_FLAG (hspi1.hdmarx, __HAL_DMA_GET_DME_FLAG_INDEX(hspi1.hdmarx));
    __HAL_DMA_CLEAR_FLAG (hspi1.hdmarx, __HAL_DMA_GET_FE_FLAG_INDEX(hspi1.hdmarx));
    /*(2.2) 清除tx的标志位*/
    __HAL_DMA_CLEAR_FLAG (hspi1.hdmatx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmatx));
    __HAL_DMA_CLEAR_FLAG (hspi1.hdmatx, __HAL_DMA_GET_HT_FLAG_INDEX(hspi1.hdmatx));
    __HAL_DMA_CLEAR_FLAG (hspi1.hdmatx, __HAL_DMA_GET_TE_FLAG_INDEX(hspi1.hdmatx));
    __HAL_DMA_CLEAR_FLAG (hspi1.hdmatx, __HAL_DMA_GET_DME_FLAG_INDEX(hspi1.hdmatx));
    __HAL_DMA_CLEAR_FLAG (hspi1.hdmatx, __HAL_DMA_GET_FE_FLAG_INDEX(hspi1.hdmatx));

    /**(3)   设置数据地址和长度**/
    /*(3.1) 设置rx的数据地址和长度*/
    hdma_spi1_rx.Instance->M0AR = rx_buf;
    __HAL_DMA_SET_COUNTER(&hdma_spi1_rx, ndtr);
    /*(3.2) 设置tx的数据地址和长度*/
    hdma_spi1_tx.Instance->M0AR = tx_buf;
    __HAL_DMA_SET_COUNTER(&hdma_spi1_tx, ndtr);

    /**(4) 使能DMA(感觉上面三步是又做了一次初始化)**/
    __HAL_DMA_ENABLE(&hdma_spi1_rx);
    __HAL_DMA_ENABLE(&hdma_spi1_tx);
}


