#include "BMI088Middleware.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_delay.h"


/**[1.1]
 * brief				��ʼ��BMI088��GPIO
 * postscript			HAL���ʼ���Ѿ����ã������ǿպ��������ʹ��ģ��I2C������������г�ʼ��*/
void BMI088_GPIO_init(void)
{

}
/**[1.2]
 * brief				��ʼ��BMI088��ͨ�Žӿ�
 * postscript			HAL���ʼ���Ѿ����ã������ǿպ��������ʹ��ģ��I2C������������г�ʼ��*/
void BMI088_com_init(void)
{


}

/**[2]
 * brief				BMI088��д����
 * retval              ���ؽ��յ�������
 * postscript			����hal�⺯��*/
extern SPI_HandleTypeDef hspi1;
uint8_t BMI088_read_write_byte(uint8_t txdata)
{
    uint8_t rx_data;
    HAL_SPI_TransmitReceive(&hspi1, &txdata, &rx_data, 1, 1000);
    return rx_data;
}
/**[3.1]
 * brief					���ٶȼ�Ƭѡ�ź�������
 * postscript			*/
void BMI088_ACCEL_NS_L(void)
{
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
}
/**[3.2]
 * brief					���ٶȼ�Ƭѡ�ź�������
 * postscript			*/
void BMI088_ACCEL_NS_H(void)
{
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
}
/**[3.3]
 * brief					������Ƭѡ�ź�������
 * postscript			*/
void BMI088_GYRO_NS_L(void)
{
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
}
/**[3.4]
 * brief					������Ƭѡ�ź�������
 * postscript			*/
void BMI088_GYRO_NS_H(void)
{
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
}

/**[4.1]
 * brief					��ʱx����
 * param[in]     Ҫ��ʱ�ĺ�����
 * postscript			�����ʱ��ist8310����ʱʲô����*/
void BMI088_delay_ms(uint16_t ms)
{

    osDelay(ms);
}
/**[4.2]
 * brief					��ʱx΢��
 * param[in]     Ҫ��ʱ��΢����
 * postscript			�����ʱ��ist8310����ʱʲô����*/
void BMI088_delay_us(uint16_t us)
{
    delay_us(us);
}


