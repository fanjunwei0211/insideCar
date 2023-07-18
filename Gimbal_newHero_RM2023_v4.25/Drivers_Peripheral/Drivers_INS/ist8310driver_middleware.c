/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       IST8310middleware.c/h
  * @brief      IST8310�������м�㣬���IST8310��IICͨ��,��Ϊ����MPU6500��SPIͨ��
  *             �������õ���ͨ��mpu6500��IIC_SLV0��ɶ�ȡ��IIC_SLV4���д�롣
  * @note       IST8310ֻ֧��IIC��ȡ
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
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
 * brief				��ʼ��IST8310��GPIO
 * postscript			HAL���ʼ���Ѿ����ã������ǿպ��������ʹ��ģ��I2C������������г�ʼ��*/
void ist8310_GPIO_init(void)
{
}
/**[1.2]
 * brief				��ʼ��IST8310��ͨ�Žӿ�
 * postscript			HAL���ʼ���Ѿ����ã������ǿպ��������ʹ��ģ��I2C������������г�ʼ��*/
void ist8310_com_init(void)
{
}
/**[2.1]
 * brief					ͨ��I2C��ȡIST8310��һ���ֽ�
 * param					�Ĵ�����ַ
 * retval				�Ĵ����������
 * postscript		����I2C��hal�⺯��	�����ǵ�ַΪʲôҪ����һλ*/
extern I2C_HandleTypeDef hi2c3;
uint8_t ist8310_IIC_read_single_reg(uint8_t reg)
{
    uint8_t res;
    HAL_I2C_Mem_Read(&hi2c3, IST8310_IIC_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, &res, 1, 100);
    return res;
}
/**[2.2]
 * brief					ͨ��I2Cд��һ���ֽڵ�IST8310�ļĴ�����
 * param1				�Ĵ�����ַ
 * param2				д��ֵ
 * postscript		����I2C��hal�⺯��	�����ǵ�ַΪʲôҪ����һλ*/
void ist8310_IIC_write_single_reg(uint8_t reg, uint8_t data)
{
    HAL_I2C_Mem_Write(&hi2c3, IST8310_IIC_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);

}
/**[2.3]
 * brief					ͨ��I2C��ȡIST8310�Ķ���ֽ�
 * param[in]			�Ĵ�����ʼ��ַ
 * param[out]		��ȡ�������ȴ浽������
 * param[in]			��ȡ�ֽ�����
 * postscript		����I2C��hal�⺯��	�����ǵ�ַΪʲôҪ����һλ*/
void ist8310_IIC_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    HAL_I2C_Mem_Read(&hi2c3, IST8310_IIC_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 100);
}
/**[2.4]
 * brief					ͨ��I2Cд��IST8310�Ķ���ֽ�
 * param[in]			�Ĵ�����ʼ��ַ
 * param[out]		Ҫд������ݵ���ʼ��ַ
 * param[in]			��ȡ�ֽ�����
 * postscript		����I2C��hal�⺯��	�����ǵ�ַΪʲôҪ����һλ*/
void ist8310_IIC_write_muli_reg(uint8_t reg, uint8_t *data, uint8_t len)
{
    HAL_I2C_Mem_Write(&hi2c3, IST8310_IIC_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, data, len, 100);
}



/**[3.1]
 * brief					����RSTN����Ϊ1
 * postscript									*/
void ist8310_RST_H(void)
{
    HAL_GPIO_WritePin(RSTN_IST8310_GPIO_Port, RSTN_IST8310_Pin, GPIO_PIN_SET);
}
/**[3.2]
 * brief					����RSTN����Ϊ0
 * postscript		�͵�ƽ����ist8310			*/
extern void ist8310_RST_L(void)
{
    HAL_GPIO_WritePin(RSTN_IST8310_GPIO_Port, RSTN_IST8310_Pin, GPIO_PIN_RESET);
}

/**[4.1]
 * brief					��ʱx����
 * param[in]			ms��������
 * postscript			*/
void ist8310_delay_ms(uint16_t ms)
{
    osDelay(ms);
}
/**[4.2]
 * brief					��ʱx΢��
 * param[in]			us��Ҫ��ʱ��΢����
 * postscript				*/
void ist8310_delay_us(uint16_t us)
{
    delay_us(us);
}

