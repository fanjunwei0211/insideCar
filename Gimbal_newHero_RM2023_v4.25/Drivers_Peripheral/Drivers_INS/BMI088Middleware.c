#include "BMI088Middleware.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_delay.h"


/**[1.1]
 * brief				初始化BMI088的GPIO
 * postscript			HAL库初始化已经配置，所以是空函数。如果使用模拟I2C可以在这里进行初始化*/
void BMI088_GPIO_init(void)
{

}
/**[1.2]
 * brief				初始化BMI088的通信接口
 * postscript			HAL库初始化已经配置，所以是空函数。如果使用模拟I2C可以在这里进行初始化*/
void BMI088_com_init(void)
{


}

/**[2]
 * brief				BMI088读写操作
 * retval              返回接收到的数据
 * postscript			调用hal库函数*/
extern SPI_HandleTypeDef hspi1;
uint8_t BMI088_read_write_byte(uint8_t txdata)
{
    uint8_t rx_data;
    HAL_SPI_TransmitReceive(&hspi1, &txdata, &rx_data, 1, 1000);
    return rx_data;
}
/**[3.1]
 * brief					加速度计片选信号线拉低
 * postscript			*/
void BMI088_ACCEL_NS_L(void)
{
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
}
/**[3.2]
 * brief					加速度计片选信号线拉高
 * postscript			*/
void BMI088_ACCEL_NS_H(void)
{
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
}
/**[3.3]
 * brief					陀螺仪片选信号线拉低
 * postscript			*/
void BMI088_GYRO_NS_L(void)
{
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
}
/**[3.4]
 * brief					陀螺仪片选信号线拉高
 * postscript			*/
void BMI088_GYRO_NS_H(void)
{
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
}

/**[4.1]
 * brief					延时x毫秒
 * param[in]     要延时的毫秒数
 * postscript			这个延时和ist8310的延时什么区别*/
void BMI088_delay_ms(uint16_t ms)
{

    osDelay(ms);
}
/**[4.2]
 * brief					延时x微秒
 * param[in]     要延时的微秒数
 * postscript			这个延时和ist8310的延时什么区别*/
void BMI088_delay_us(uint16_t us)
{
    delay_us(us);
}


