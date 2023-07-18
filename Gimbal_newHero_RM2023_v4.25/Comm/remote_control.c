#include "remote_control.h"
#include "main.h"//不加不认识UART_HandleTypeDef
#include "bsp_rc.h"

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

/**[1]						
  * brief						遥控器初始化，其实就是设置了一下串口接收到的遥控器数据存到内存哪两个变量里
	* postscript		  详细的配置函数RC_init()在“bsp_rc.c”*/
static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];
void remote_control_init()
{
		RC_init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
}




/**[2]						
  * brief						串口3中断处理函数
	* postscript		  为什么要判断缓存区？判断现在的缓存区是Memory0还是Memory1,如果是0则处理1中的数据，如果是1则处理0中的信息，处理时不会影响另一个缓冲区进行数据的接收：*/
RC_ctrl_t rc_ctrl;
void USART3_IRQHandler(void)
{
		/*（1）	如果是接收中断*/
    if(huart3.Instance->SR & UART_FLAG_RXNE)//如果是接收中断			SR（Status register,状态寄存器）	RXNE（RX Not Empty），如果接收到数据
    {
        __HAL_UART_CLEAR_PEFLAG(&huart3);//进行寄存器中断标志位的处理，PEFLAG（PE flag，PE好像是原理图上引脚的名字，连着中断好像）
    }
		
		/*（2）	如果是空闲中断*/
    else if(USART3->SR & UART_FLAG_IDLE)//如果是空闲中断		idle（空闲的?）		串口接收完一帧数据（一帧数据由多个字节组成，自己设置一次接收多少数据），IDLE位会置1。
    {
        static uint16_t this_time_rx_len = 0;
				
				/*（2.1）	处理中断标志位*/
        __HAL_UART_CLEAR_PEFLAG(&huart3);

				/*（2.2）判断进行接收的缓冲区是1号缓冲区还是2号缓冲区，并把数据存到相应缓冲区*/
			
					/*（2.2.1） 如果是1号缓冲区*/
        if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)	//通过这个CT位判断，盲猜这个CR寄存器里这个CT位不是事先设定好就不变的，而是随着缓冲区的切换而变的
        {
            /*（2.2.1.1）	失效DMA*/
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

						/*（2.2.1.2）	获取本次DMA接收到的数据长度,长度 = 设定长度 - 剩余长度*/
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;//NDTR（number of data items to transfer register，要传输的数据项数目）

						/*（2.2.1.3）	重新设定数据长度,以便下次继续用*/
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

						/*（2.2.1.4）	修改CR寄存器的CT位，CT位置1，切换到缓冲区2*/
            hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;//如果接受数据填满缓冲区，会自动切换缓冲区。这里设置一个缓冲区的长度是一帧数据长度的两倍，肯定填不满，所以需要手动切换缓冲区。
            
            /*（2.2.1.5）	都改完再使能DMA*/
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

						/*（2.2.1.6）	遥控器数据解码*/
            if(this_time_rx_len == RC_FRAME_LENGTH)//判断是否与一帧数据（18 字节）长度相等，如果相等则调用函数 sbus_to_rc 进行遥控器数据的解码。
            {
                sbus_to_rc(sbus_rx_buf[0], &rc_ctrl);
            }
        }
				
					/*（2.2.2） 如果是2号缓冲区*/
        else
        {
            /*（2.2.2.1） 	失效DMA*/
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

						/*（2.2.2.2）	获取本次DMA接收到的数据长度,长度 = 设定长度 - 剩余长度*/
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            /*（2.2.2.3）	重新设定数据长度,以便下次继续用*/
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            /*（2.2.2.4）	修改CR寄存器的CT位，CT位置0，切换到缓冲区1*/
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
            
            /*（2.2.2.5）	都改完再使能DMA*/
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

						/*（2.2.2.6）	遥控器数据解码*/
            if(this_time_rx_len == RC_FRAME_LENGTH)//判断是否与一帧数据（18 字节）长度相等，如果相等则调用函数 sbus_to_rc 进行遥控器数据的解码。
            {
                sbus_to_rc(sbus_rx_buf[1], &rc_ctrl);
            }
        }
    }
}

/**[3]						
  * brief						遥控器数据解码
	*	param[in]				sbus_buf:存放遥控器原始数据的内存变量。遥控器发送数据到串口，串口通过DMA把数据发送到内存缓冲区
	*	param[out]			rc_ctrl:遥控器结构体变量，解析完的数据存放在这个结构体里面
	* postscript		  */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }
		/*（1）	摇杆数据*/
    rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
    rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 2
                         (sbus_buf[4] << 10)) &0x07ff;
    rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
		
		/*（2）	拨杆数据*/
    rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                  //!< Switch left
    rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                       //!< Switch right
		
		/*（3）	鼠标数据*/
    rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< Mouse X axis
    rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis
    rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis
    rc_ctrl->mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press ?
    rc_ctrl->mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press ?
		
		/*（4）	键盘数据？*/
    rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value
		
		/*（5）	是不是遥控器侧轮数据？*/
    rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 //NULL

		/*（6）	改一下遥控器数据范围*/
    rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;//减1024让遥控器上下摇杆数值在-660到+660
    rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
}
