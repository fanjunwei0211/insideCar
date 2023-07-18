/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       IST8310.c/h
  * @brief      IST8310磁力计驱动函数，包括初始化函数，处理数据函数，通信读取函数
  *             本工程是将MPU6500 IIC_SLV0设置为自动读取IST8310数据，读取
  *             MPU_EXT_SENS_DATA_00保存了IST8310的Status，通过判断标志位，来更新
  *             数据。
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

#include "ist8310driver.h"
#include "ist8310driver_middleware.h"



/**[1]						
  * brief					初始化IST8310
	* retval				返回错误信息
	* postscript																													*/
#define IST8310_WHO_AM_I 0x00       //ist8310 who am I 寄存器
#define IST8310_WHO_AM_I_VALUE 0x10 //设备 ID
#define IST8310_WRITE_REG_NUM 4 //IST8310需要设置的寄存器数目
static const	uint8_t ist8310_write_reg_data_error[IST8310_WRITE_REG_NUM][3]={
		//寄存器地址	写入的值	返回的错误码
	{			0x0B,			0x08,			0x01		},			//开启中断，并设置低电平
	{			0x41,			0x09,			0x02		},			//平均采样两次
	{			0x42,			0xC0,			0x03		},			//这个寄存器必须设置成0xC0
	{			0x0A,			0x0B,			0x04		},			//200Hz输出频率
};
uint8_t ist8310_init(void)
{
    static const uint8_t wait_time = 1;
    static const uint8_t sleepTime = 50;
    uint8_t res = 0;
    uint8_t writeNum = 0;
		/*(1) 初始化GPIO和通信*/
    ist8310_GPIO_init();
    ist8310_com_init();// 两个空函数，因为hal库初始化已经配置，如果使用模拟I2C可以自己写
		/*(2) 重启设备*/
    ist8310_RST_L(); // 电平拉低复位重启
    ist8310_delay_ms(sleepTime);// 延时50ms长不长
    ist8310_RST_H(); // 再拉高
    ist8310_delay_ms(sleepTime);// 再延时50ms
		/*(3) 验证设备ID*/
    res = ist8310_IIC_read_single_reg(IST8310_WHO_AM_I);// 读who_am_i寄存器，并把里面的值赋给res
    if (res != IST8310_WHO_AM_I_VALUE)
    {
        return IST8310_NO_SENSOR;
    }
    ist8310_delay_ms(wait_time);
    /*(4) 配置IST8310*/
    for (writeNum = 0; writeNum < IST8310_WRITE_REG_NUM; writeNum++)
    {
				/*(4.1) 向寄存器写值*/
        ist8310_IIC_write_single_reg(ist8310_write_reg_data_error[writeNum][0], ist8310_write_reg_data_error[writeNum][1]);
        ist8310_delay_ms(wait_time);
				/*(4.2) 判断写进去的对不对*/
        res = ist8310_IIC_read_single_reg(ist8310_write_reg_data_error[writeNum][0]);
        ist8310_delay_ms(wait_time);
        if (res != ist8310_write_reg_data_error[writeNum][1])
        {
            return ist8310_write_reg_data_error[writeNum][2];
        }
    }

    return IST8310_NO_ERROR;
}


/**[2]						
  * brief			读取磁场数据	
  *	param[out]		磁场数组
  * postscript																													*/
#define MAG_SEN 0.3f //转换成 uT
void ist8310_read_mag(fp32 mag[3])
{
    uint8_t buf[6];
    int16_t temp_ist8310_data = 0;
    ist8310_IIC_read_muli_reg(0x03, buf, 6);

    temp_ist8310_data = (int16_t)((buf[1] << 8) | buf[0]);
    mag[0] = MAG_SEN * temp_ist8310_data;
    temp_ist8310_data = (int16_t)((buf[3] << 8) | buf[2]);
    mag[1] = MAG_SEN * temp_ist8310_data;
    temp_ist8310_data = (int16_t)((buf[5] << 8) | buf[4]);
    mag[2] = MAG_SEN * temp_ist8310_data;
}
/**[3]
 * brief			如果已经通过I2C的DMA方式读取到了从STA1到DATAZL的数据，可以使用这个函数进行处理
 * param[in]		数据指针，从STA1(0x02)寄存器 到DATAZL(0x08)寄存器
 * param[out]		ist8310的数据结构
 * postscript											*/
void ist8310_read_over(uint8_t *status_buf, ist8310_real_data_t *ist8310_real_data)
{

    if (status_buf[0] & 0x01)// 通过判断STAT1寄存器的位判断有没有新的数据产生
    {
        int16_t temp_ist8310_data = 0;
				/*(1) 修改状态位*/
        ist8310_real_data->status |= 1 << IST8310_DATA_READY_BIT;
				/*(2) 数据拼接*/
        temp_ist8310_data = (int16_t)((status_buf[2] << 8) | status_buf[1]);
        ist8310_real_data->mag[0] = MAG_SEN * temp_ist8310_data;
        temp_ist8310_data = (int16_t)((status_buf[4] << 8) | status_buf[3]);
        ist8310_real_data->mag[1] = MAG_SEN * temp_ist8310_data;
        temp_ist8310_data = (int16_t)((status_buf[6] << 8) | status_buf[5]);
        ist8310_real_data->mag[2] = MAG_SEN * temp_ist8310_data;
    }
    else// 否则状态位置零
    {
        ist8310_real_data->status &= ~(1 << IST8310_DATA_READY_BIT);
    }
}
