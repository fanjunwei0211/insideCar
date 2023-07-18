#include "BMI088driver.h"
#include "BMI088reg.h"
#include "BMI088Middleware.h"






#if defined(BMI088_USE_SPI)//如果使用SPI通信

/********************[1] 寄存器读写操作函数****************************/
/**[1.1]						
  * brief					向BMI088寄存器写一个字节
	* param[in]				寄存器地址
	* param[in]				要写入的数据
	* postscript		    为什么这么写？先发送寄存器地址再发送数据，难道这是协议？*/
static void BMI088_write_single_reg(uint8_t reg, uint8_t data)
{
    BMI088_read_write_byte(reg);
    BMI088_read_write_byte(data);
}
/**[1.2]						
  * brief					从BMI088寄存器读一个字节
	* param[in]				寄存器地址
	* param[out]			读出的数据放在这个变量里
	* postscript		    为什么这么写？先发送寄存器地址读出的就是整个寄存器的数据，难道这是协议？*/
static void BMI088_read_single_reg(uint8_t reg, uint8_t *return_data)
{
    BMI088_read_write_byte(reg | 0x80);
    *return_data = BMI088_read_write_byte(0x55);
}
///**[1.3]						
//  * brief					向BMI088寄存器写多个字节
//	* param[in]			寄存器地址
//	* param[in]			要写入的数据
//  * param[in]     要写入的数据长度
//	* postscript	  用指针实现给定长度数据的写入；官方把这个注释掉了没用上倒是*/
//static void BMI088_write_muli_reg(uint8_t reg, uint8_t* buf, uint8_t len )
//{
//    BMI088_read_write_byte( reg );
//    while( len != 0 )
//    {

//        BMI088_read_write_byte( *buf );
//        buf ++;
//        len --;
//    }

//}
/**[1.4]						
  * brief					从BMI088寄存器读多个字节
	* param[in]			寄存器地址
	* param[out]		读出的数据放在这个指针指向的空间里
  * param[in]     要读取的数据长度
	* postscript    用指针实现给定长度数据的读取*/
static void BMI088_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    BMI088_read_write_byte(reg | 0x80);

    while (len != 0)
    {

        *buf = BMI088_read_write_byte(0x55);
        buf++;
        len--;
    }
}

/********************[2] 加速度计读写操作函数(宏定义)****************************/
/**[2.1]						
  * brief					向加速度计写一个字节
	* param[in]				寄存器地址
	* param[in]				要写入的数据
	* postscript		    先把片选线拉低选中设备，写完再拉高		*/
#define BMI088_accel_write_single_reg(reg, data) \
    {                                            \
        BMI088_ACCEL_NS_L();                     \
        BMI088_write_single_reg((reg), (data));  \
        BMI088_ACCEL_NS_H();                     \
    }
/**[2.2]						
  * brief					从加速度计读一个字节
	* param[in]				寄存器地址
	* param[out]			读出的数据放在这个变量里
	* postscript		    为什么不直接调用函数[1.2],而且还发了两次0x55;为什么这个data不用成指针的形式				*/
#define BMI088_accel_read_single_reg(reg, data) \
    {                                           \
        BMI088_ACCEL_NS_L();                    \
        BMI088_read_write_byte((reg) | 0x80);   \
        BMI088_read_write_byte(0x55);           \
        (data) = BMI088_read_write_byte(0x55);  \
        BMI088_ACCEL_NS_H();                    \
    }
/**[2.3]						
  * brief				 向加速度计写多个字节
	* param[in]		 寄存器地址
	* param[in]		 要写入的数据
  * param[in]    要写入的数据长度
	* postscript	 这个貌似没用到，原来是注释掉的*/
#define BMI088_accel_write_muli_reg(reg, data, len) \
  {                                                 \
    BMI088_ACCEL_NS_L();                            \
    BMI088_write_muli_reg(reg, data, len);          \
    BMI088_ACCEL_NS_H();                            \
  }
/**[2.4]
  * brief					从加速度计读多个字节
  * param[in]			寄存器地址
  * param[out]		读出的数据放在这个指针指向的空间里
  * param[in]     要读取的数据长度
  * postscript    BMI088_read_muli_reg()的定义里有“BMI088_read_write_byte(reg|0x80);”这里为什么要重复写一句？
  * postscript    BMI088_read_muli_reg()的第二个参数是指针型，传入的data参数应该是指针或数组名*/
#define BMI088_accel_read_muli_reg(reg, data, len) \
    {                                              \
        BMI088_ACCEL_NS_L();                       \
        BMI088_read_write_byte((reg) | 0x80);      \
        BMI088_read_muli_reg(reg, data, len);      \
        BMI088_ACCEL_NS_H();                       \
    }
/********************[3] 陀螺仪读写操作函数****************************/
/**[3.1]						
  * brief					向陀螺仪写一个字节
	* param[in]			寄存器地址
	* param[in]			要写入的数据
	* postscript    先把片选线拉低选中设备，写完再拉高*/
#define BMI088_gyro_write_single_reg(reg, data) \
    {                                           \
        BMI088_GYRO_NS_L();                     \
        BMI088_write_single_reg((reg), (data)); \
        BMI088_GYRO_NS_H();                     \
    }
/**[3.2]						
  * brief					从陀螺仪读一个字节
	* param[in]			寄存器地址
	* param[out]		读出的数据放在这个变量里
	* postscript	  直接调用的函数[1.2]*/
#define BMI088_gyro_read_single_reg(reg, data)  \
    {                                           \
        BMI088_GYRO_NS_L();                     \
        BMI088_read_single_reg((reg), &(data)); \
        BMI088_GYRO_NS_H();                     \
    }
/**[3.3]						
  * brief					向陀螺仪写多个字节
	* param[in]			寄存器地址
	* param[in]			要写入的数据
  * param[in]     要写入的数据长度
	* postscript	  官方把这个函数注释掉了没用倒是*/
#define BMI088_gyro_write_muli_reg(reg, data, len) \
  {                                                \
    BMI088_GYRO_NS_L();                            \
    BMI088_write_muli_reg((reg), (data), (len));   \
    BMI088_GYRO_NS_H();                            \
  }
/**[3.4]						
  * brief					从陀螺仪读多个字节
	* param[in]			寄存器地址
	* param[out]		读出的数据放在这个指针指向的空间里
  * param[in]     要读取的数据长度
	* postscript		    */
#define BMI088_gyro_read_muli_reg(reg, data, len)   \
    {                                               \
        BMI088_GYRO_NS_L();                         \
        BMI088_read_muli_reg((reg), (data), (len)); \
        BMI088_GYRO_NS_H();                         \
    }


#elif defined(BMI088_USE_IIC)
/*IIC官方没写，我也不写*/     

#endif



/********************[4] 初始化****************************/
/**[4]						
  * brief				    bmi088初始化
	* retval				  返回错误信息
	* postscript		   													*/
uint8_t BMI088_init(void)
{
    uint8_t error = BMI088_NO_ERROR;
	
    /*(1)	GPIO和SPI初始化*/
    BMI088_GPIO_init();
    BMI088_com_init();

    /*(2)	如果加速度计自检通过就初始化*/
    if (bmi088_accel_self_test() != BMI088_NO_ERROR)
    {
        error |= BMI088_SELF_TEST_ACCEL_ERROR;
    }
    else
    {
        error |= bmi088_accel_init();
    }
		/*(3)	如果陀螺仪自检通过就初始化*/
    if (bmi088_gyro_self_test() != BMI088_NO_ERROR)
    {
        error |= BMI088_SELF_TEST_GYRO_ERROR;
    }
    else
    {
        error |= bmi088_gyro_init();
    }

    return error;
}
/**[4.1]						
  * brief				    加速度计自检
	* retval				  返回错误信息
	* postscript		   													*/
bool_t bmi088_accel_self_test(void)
{

    int16_t self_test_accel[2][3];

    uint8_t buf[6] = {0, 0, 0, 0, 0, 0};
    volatile uint8_t res = 0;

    uint8_t write_reg_num = 0;

    static const uint8_t write_BMI088_ACCEL_self_test_Reg_Data_Error[6][3] =
        {
            {BMI088_ACC_CONF, BMI088_ACC_NORMAL | BMI088_ACC_1600_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_ERROR},
            {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},
            {BMI088_ACC_RANGE, BMI088_ACC_RANGE_24G, BMI088_ACC_RANGE_ERROR},
            {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR},
            {BMI088_ACC_SELF_TEST, BMI088_ACC_SELF_TEST_POSITIVE_SIGNAL, BMI088_ACC_PWR_CONF_ERROR},
            {BMI088_ACC_SELF_TEST, BMI088_ACC_SELF_TEST_NEGATIVE_SIGNAL, BMI088_ACC_PWR_CONF_ERROR}

        };

    //check commiunication is normal
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    // reset  bmi088 accel sensor and wait for > 50ms
    BMI088_accel_write_single_reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    BMI088_delay_ms(BMI088_LONG_DELAY_TIME);

    //check commiunication is normal
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    if (res != BMI088_ACC_CHIP_ID_VALUE)
    {
        return BMI088_NO_SENSOR;
    }

    // set the accel register
    for (write_reg_num = 0; write_reg_num < 4; write_reg_num++)
    {

        BMI088_accel_write_single_reg(write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num][0], write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num][1]);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        BMI088_accel_read_single_reg(write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num][0], res);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
			
        if (res != write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num][1])
        {
            return write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num][2];
        }
        // accel conf and accel range  . the two register set need wait for > 50ms
        BMI088_delay_ms(BMI088_LONG_DELAY_TIME);
    }

    // self test include postive and negative
    for (write_reg_num = 0; write_reg_num < 2; write_reg_num++)
    {

        BMI088_accel_write_single_reg(write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num + 4][0], write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num + 4][1]);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        BMI088_accel_read_single_reg(write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num + 4][0], res);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if (res != write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num + 4][1])
        {
            return write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num + 4][2];
        }
        // accel conf and accel range  . the two register set need wait for > 50ms
        BMI088_delay_ms(BMI088_LONG_DELAY_TIME);

        // read response accel
        BMI088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);

        self_test_accel[write_reg_num][0] = (int16_t)((buf[1]) << 8) | buf[0];
        self_test_accel[write_reg_num][1] = (int16_t)((buf[3]) << 8) | buf[2];
        self_test_accel[write_reg_num][2] = (int16_t)((buf[5]) << 8) | buf[4];
    }

    //set self test off
    BMI088_accel_write_single_reg(BMI088_ACC_SELF_TEST, BMI088_ACC_SELF_TEST_OFF);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_accel_read_single_reg(BMI088_ACC_SELF_TEST, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    if (res != (BMI088_ACC_SELF_TEST_OFF))
    {
        return BMI088_ACC_SELF_TEST_ERROR;
    }

    //reset the accel sensor
    BMI088_accel_write_single_reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    BMI088_delay_ms(BMI088_LONG_DELAY_TIME);

    if ((self_test_accel[0][0] - self_test_accel[1][0] < 1365) || (self_test_accel[0][1] - self_test_accel[1][1] < 1365) || (self_test_accel[0][2] - self_test_accel[1][2] < 680))
    {
        return BMI088_SELF_TEST_ACCEL_ERROR;
    }

    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    return BMI088_NO_ERROR;
}
/**[4.2]						
  * brief				    加速度计初始化
	* retval				  返回错误信息
	* postscript		   													*/
static uint8_t write_BMI088_accel_reg_data_error[BMI088_WRITE_ACCEL_REG_NUM][3] =
    {
        {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},
        {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR},
        {BMI088_ACC_CONF,  BMI088_ACC_NORMAL| BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_ERROR},
        {BMI088_ACC_RANGE, BMI088_ACC_RANGE_3G, BMI088_ACC_RANGE_ERROR},
        {BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW, BMI088_INT1_IO_CTRL_ERROR},
        {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT, BMI088_INT_MAP_DATA_ERROR}

};

bool_t bmi088_accel_init(void)
{
    volatile uint8_t res = 0;
    uint8_t write_reg_num = 0;

		/**(1) 读“who am I”寄存器 检查加速度计是否正常 **/

		/*(1.1) 检查通信*/
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);// 读一下“who am I”寄存器的值，存到变量res
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);// 干嘛还要读一次？
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME); 				// 这第一次读完res的值，它这也没判断res对不对啊，还有必要检查嘛

    /*(1.2) 复位*/
    BMI088_accel_write_single_reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    BMI088_delay_ms(BMI088_LONG_DELAY_TIME);

    /*(1.3) 复位之后再检查一遍通信是否正常*/
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

		/*(1.4) 检查“who am I”寄存器判断一下有没有读出来或者读出来的值对不对*/
    if (res != BMI088_ACC_CHIP_ID_VALUE)
    {
        return BMI088_NO_SENSOR;
    }

    /**(2) 配置加速度传感器并检查 **/
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_ACCEL_REG_NUM; write_reg_num++)
    {
				/*(2.1) 轮流往寄存器里写值*/
        BMI088_accel_write_single_reg(write_BMI088_accel_reg_data_error[write_reg_num][0], write_BMI088_accel_reg_data_error[write_reg_num][1]);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
				/*(2.2) 写完再把寄存器里的值读出来*/
        BMI088_accel_read_single_reg(write_BMI088_accel_reg_data_error[write_reg_num][0], res);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
				/*(2.3) 判断一下写进去和读出来的值一不一样*/
        if (res != write_BMI088_accel_reg_data_error[write_reg_num][1])
        {
            return write_BMI088_accel_reg_data_error[write_reg_num][2];
        }
    }
    return BMI088_NO_ERROR;// 如果都没返回错误值就返回没错误
}
/**[4.3]						
  * brief				    陀螺仪自检
	* retval				  返回错误信息
	* postscript		   													*/
bool_t bmi088_gyro_self_test(void)
{
    uint8_t res = 0;
    uint8_t retry = 0;
    //check commiunication is normal
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    //reset the gyro sensor
    BMI088_gyro_write_single_reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    BMI088_delay_ms(BMI088_LONG_DELAY_TIME);
    //check commiunication is normal after reset
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    BMI088_gyro_write_single_reg(BMI088_GYRO_SELF_TEST, BMI088_GYRO_TRIG_BIST);
    BMI088_delay_ms(BMI088_LONG_DELAY_TIME);

    do
    {

        BMI088_gyro_read_single_reg(BMI088_GYRO_SELF_TEST, res);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
        retry++;
    } while (!(res & BMI088_GYRO_BIST_RDY) && retry < 10);

    if (retry == 10)
    {
        return BMI088_SELF_TEST_GYRO_ERROR;
    }

    if (res & BMI088_GYRO_BIST_FAIL)
    {
        return BMI088_SELF_TEST_GYRO_ERROR;
    }

    return BMI088_NO_ERROR;
}
/**[4.4]						
  * brief				    陀螺仪初始化
	* retval				  返回错误信息
	* postscript		   													*/
static uint8_t write_BMI088_gyro_reg_data_error[BMI088_WRITE_GYRO_REG_NUM][3] =
    {
        {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR},
        {BMI088_GYRO_BANDWIDTH, BMI088_GYRO_1000_116_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set, BMI088_GYRO_BANDWIDTH_ERROR},
        {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR},
        {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR},
        {BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW, BMI088_GYRO_INT3_INT4_IO_CONF_ERROR},
        {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, BMI088_GYRO_INT3_INT4_IO_MAP_ERROR}

};
bool_t bmi088_gyro_init(void)
{
    uint8_t write_reg_num = 0;
    uint8_t res = 0;

		/**(1) 读“who am I”寄存器 检查陀螺仪是否正常 **/
		/*(1.1) 检查通信*/
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    /*(1.2) 复位*/
    BMI088_gyro_write_single_reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    BMI088_delay_ms(BMI088_LONG_DELAY_TIME);
    /*(1.3) 复位之后再检查一遍通信是否正常*/
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    /*(1.4) 判断一下有没有读出来或者读出来的值对不对*/
    if (res != BMI088_GYRO_CHIP_ID_VALUE)
    {
        return BMI088_NO_SENSOR;
    }

    /**(2) 配置加速度传感器并检查 **/
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_GYRO_REG_NUM; write_reg_num++)
    {
				/*(2.1) 轮流往寄存器里写值*/
        BMI088_gyro_write_single_reg(write_BMI088_gyro_reg_data_error[write_reg_num][0], write_BMI088_gyro_reg_data_error[write_reg_num][1]);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
				/*(2.2) 写完再把寄存器里的值读出来*/
        BMI088_gyro_read_single_reg(write_BMI088_gyro_reg_data_error[write_reg_num][0], res);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
				/*(2.3) 判断一下写进去和读出来的值一不一样*/
        if (res != write_BMI088_gyro_reg_data_error[write_reg_num][1])
        {
            return write_BMI088_gyro_reg_data_error[write_reg_num][2];
        }
    }

    return BMI088_NO_ERROR;// 如果都没返回错误值就返回没错误
}

/********************[5] 读取角速度和加速度****************************/
/**[5.1]						
  * brief				    读取角速度，加速度，温度
	* param[out]      三个欧拉角？
  * param[out]      三个方向加速度
  * param[out]      温度
	* postscript		  函数功能主要包括寄存器读取和数据拼接两部分					*/
fp32 BMI088_ACCEL_SEN = BMI088_ACCEL_3G_SEN;
fp32 BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;
void BMI088_read(fp32 gyro[3], fp32 accel[3], fp32 *temperate)
{
    uint8_t buf[8] = {0, 0, 0, 0, 0, 0};
    int16_t bmi088_raw_temp;
		/**(1) 读取加速度计数据 **/
		/*(1.1) 加速度计寄存器读取*/
    BMI088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);
		/*(1.2) 数据拼接*/
    bmi088_raw_temp = (int16_t)((buf[1]) << 8) | buf[0];
    accel[0] = bmi088_raw_temp * BMI088_ACCEL_SEN;
    bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
    accel[1] = bmi088_raw_temp * BMI088_ACCEL_SEN;
    bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
    accel[2] = bmi088_raw_temp * BMI088_ACCEL_SEN;
		/**(2) 读取陀螺仪数据 **/
		/*(2.1) 陀螺仪寄存器读取*/
    BMI088_gyro_read_muli_reg(BMI088_GYRO_CHIP_ID, buf, 8);
		/*(2.2) 数据拼接*/
    if(buf[0] == BMI088_GYRO_CHIP_ID_VALUE)
    {
        bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
        gyro[0] = bmi088_raw_temp * BMI088_GYRO_SEN;
        bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
        gyro[1] = bmi088_raw_temp * BMI088_GYRO_SEN;
        bmi088_raw_temp = (int16_t)((buf[7]) << 8) | buf[6];
        gyro[2] = bmi088_raw_temp * BMI088_GYRO_SEN;
    }
	  /**(3) 读取温度数据 **/
		/*(3.1) 加速度计寄存器读取（温度数据在加速度计寄存器里面）*/
    BMI088_accel_read_muli_reg(BMI088_TEMP_M, buf, 2);
		/*(3.2) 数据拼接）*/
    bmi088_raw_temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));

    if (bmi088_raw_temp > 1023)
    {
        bmi088_raw_temp -= 2048;
    }

    *temperate = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
}


/**[5.2]						
  * brief				    读取温度并转换
	* param[in]      	传感器原始温度数据
  * param[out]      实际温度
* postscript		  函数功能主要包括寄存器读取和数据拼接两部分			（姿态解算程序后来又加上的）		*/
void BMI088_temperature_read_over(uint8_t *rx_buf, fp32 *temperate)
{
    int16_t bmi088_raw_temp;
    bmi088_raw_temp = (int16_t)((rx_buf[0] << 3) | (rx_buf[1] >> 5));

    if (bmi088_raw_temp > 1023)
    {
        bmi088_raw_temp -= 2048;
    }
    *temperate = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;

}
/**[5.3]						
  * brief				    读取加速度并转换
	* param[in]     	传感器原始加速度数据
  * param[out]      实际温度
	* param[out]				??
	* postscript		  函数功能主要包括寄存器读取和数据拼接两部分				（姿态解算程序后来又加上的）	*/
void BMI088_accel_read_over(uint8_t *rx_buf, fp32 accel[3], fp32 *time)
{
    int16_t bmi088_raw_temp;
    uint32_t sensor_time;
    bmi088_raw_temp = (int16_t)((rx_buf[1]) << 8) | rx_buf[0];
    accel[0] = bmi088_raw_temp * BMI088_ACCEL_SEN;
    bmi088_raw_temp = (int16_t)((rx_buf[3]) << 8) | rx_buf[2];
    accel[1] = bmi088_raw_temp * BMI088_ACCEL_SEN;
    bmi088_raw_temp = (int16_t)((rx_buf[5]) << 8) | rx_buf[4];
    accel[2] = bmi088_raw_temp * BMI088_ACCEL_SEN;
    sensor_time = (uint32_t)((rx_buf[8] << 16) | (rx_buf[7] << 8) | rx_buf[6]);
    *time = sensor_time * 39.0625f;

}
/**[5.4]						
  * brief				    读取角速度
	* param[in]      传感器原始角速度数据
  * param[out]      三个方向角速度
	* postscript		  函数功能主要包括寄存器读取和数据拼接两部分			（姿态解算程序后来又加上的）		*/
void BMI088_gyro_read_over(uint8_t *rx_buf, fp32 gyro[3])
{
    int16_t bmi088_raw_temp;
    bmi088_raw_temp = (int16_t)((rx_buf[1]) << 8) | rx_buf[0];
    gyro[0] = bmi088_raw_temp * BMI088_GYRO_SEN;
    bmi088_raw_temp = (int16_t)((rx_buf[3]) << 8) | rx_buf[2];
    gyro[1] = bmi088_raw_temp * BMI088_GYRO_SEN;
    bmi088_raw_temp = (int16_t)((rx_buf[5]) << 8) | rx_buf[4];
    gyro[2] = bmi088_raw_temp * BMI088_GYRO_SEN;
}





/******************************[6]	这些函数貌似没用到************************************************/
void BMI088_read_gyro_who_am_i(void)
{
    uint8_t buf;
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, buf);
}


void BMI088_read_accel_who_am_i(void)
{
    volatile uint8_t buf;
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, buf);
    buf = 0;

}

uint32_t get_BMI088_sensor_time(void)
{
    uint32_t sensor_time = 0;
    uint8_t buf[3];
    BMI088_accel_read_muli_reg(BMI088_SENSORTIME_DATA_L, buf, 3);

    sensor_time = (uint32_t)((buf[2] << 16) | (buf[1] << 8) | (buf[0]));

    return sensor_time;
}

fp32 get_BMI088_temperate(void)
{
    uint8_t buf[2];
    fp32 temperate;
    int16_t temperate_raw_temp;

    BMI088_accel_read_muli_reg(BMI088_TEMP_M, buf, 2);

    temperate_raw_temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));

    if (temperate_raw_temp > 1023)
    {
        temperate_raw_temp -= 2048;
    }

    temperate = temperate_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;

    return temperate;
}

void get_BMI088_gyro(int16_t gyro[3])
{
    uint8_t buf[6] = {0, 0, 0, 0, 0, 0};
    int16_t gyro_raw_temp;

    BMI088_gyro_read_muli_reg(BMI088_GYRO_X_L, buf, 6);

    gyro_raw_temp = (int16_t)((buf[1]) << 8) | buf[0];
    gyro[0] = gyro_raw_temp ;
    gyro_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
    gyro[1] = gyro_raw_temp ;
    gyro_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
    gyro[2] = gyro_raw_temp ;
}

void get_BMI088_accel(fp32 accel[3])
{
    uint8_t buf[6] = {0, 0, 0, 0, 0, 0};
    int16_t accel_raw_temp;

    BMI088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);

    accel_raw_temp = (int16_t)((buf[1]) << 8) | buf[0];
    accel[0] = accel_raw_temp * BMI088_ACCEL_SEN;
    accel_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
    accel[1] = accel_raw_temp * BMI088_ACCEL_SEN;
    accel_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
    accel[2] = accel_raw_temp * BMI088_ACCEL_SEN;
}

