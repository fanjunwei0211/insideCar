/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       IST8310middleware.c/h
  * @brief      IST8310磁力计中间层，完成IST8310的IIC通信,因为设置MPU6500的SPI通信
  *             所以设置的是通过mpu6500的IIC_SLV0完成读取，IIC_SLV4完成写入。
  * @note       IST8310只支持IIC读取
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */

#include "ist8310driver_middleware.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_delay.h"
//#include "bsp_i2c.h"



/**[1.1]
 * brief				初始化IST8310的GPIO
 * postscript			HAL库初始化已经配置，所以是空函数。如果使用模拟I2C可以在这里进行初始化*/
void ist8310_GPIO_init(void)
{
}
/**[1.2]
 * brief				初始化IST8310的通信接口
 * postscript			HAL库初始化已经配置，所以是空函数。如果使用模拟I2C可以在这里进行初始化*/
void ist8310_com_init(void)
{
}
/**[2.1]
 * brief					通过I2C读取IST8310的一个字节
 * param					寄存器地址
 * retval				寄存器存的数据
 * postscript		调用I2C的hal库函数	，但是地址为什么要左移一位*/
extern I2C_HandleTypeDef hi2c3;
uint8_t ist8310_IIC_read_single_reg(uint8_t reg)
{
    uint8_t res;
    HAL_I2C_Mem_Read(&hi2c3, IST8310_IIC_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, &res, 1, 100);
    return res;
}
/**[2.2]
 * brief					通过I2C写入一个字节到IST8310的寄存器中
 * param1				寄存器地址
 * param2				写入值
 * postscript		调用I2C的hal库函数	，但是地址为什么要左移一位*/
void ist8310_IIC_write_single_reg(uint8_t reg, uint8_t data)
{
    HAL_I2C_Mem_Write(&hi2c3, IST8310_IIC_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);

}
/**[2.3]
 * brief					通过I2C读取IST8310的多个字节
 * param[in]			寄存器开始地址
 * param[out]		读取的数据先存到缓冲区
 * param[in]			读取字节总数
 * postscript		调用I2C的hal库函数	，但是地址为什么要左移一位*/
void ist8310_IIC_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    HAL_I2C_Mem_Read(&hi2c3, IST8310_IIC_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 100);
}
/**[2.4]
 * brief					通过I2C写入IST8310的多个字节
 * param[in]			寄存器开始地址
 * param[out]		要写入的数据的起始地址
 * param[in]			读取字节总数
 * postscript		调用I2C的hal库函数	，但是地址为什么要左移一位*/
void ist8310_IIC_write_muli_reg(uint8_t reg, uint8_t *data, uint8_t len)
{
    HAL_I2C_Mem_Write(&hi2c3, IST8310_IIC_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, data, len, 100);
}



/**[3.1]
 * brief					设置RSTN引脚为1
 * postscript									*/
void ist8310_RST_H(void)
{
    HAL_GPIO_WritePin(RSTN_IST8310_GPIO_Port, RSTN_IST8310_Pin, GPIO_PIN_SET);
}
/**[3.2]
 * brief					设置RSTN引脚为0
 * postscript		低电平重启ist8310			*/
extern void ist8310_RST_L(void)
{
    HAL_GPIO_WritePin(RSTN_IST8310_GPIO_Port, RSTN_IST8310_Pin, GPIO_PIN_RESET);
}

/**[4.1]
 * brief					延时x毫秒
 * param[in]			ms：毫秒数
 * postscript			*/
void ist8310_delay_ms(uint16_t ms)
{
    osDelay(ms);
}
/**[4.2]
 * brief					延时x微秒
 * param[in]			us：要延时的微秒数
 * postscript				*/
void ist8310_delay_us(uint16_t us)
{
    delay_us(us);
}

