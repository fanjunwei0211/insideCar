/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       IST8310.c/h
  * @brief      IST8310����������������������ʼ���������������ݺ�����ͨ�Ŷ�ȡ����
  *             �������ǽ�MPU6500 IIC_SLV0����Ϊ�Զ���ȡIST8310���ݣ���ȡ
  *             MPU_EXT_SENS_DATA_00������IST8310��Status��ͨ���жϱ�־λ��������
  *             ���ݡ�
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

#include "ist8310driver.h"
#include "ist8310driver_middleware.h"



/**[1]						
  * brief					��ʼ��IST8310
	* retval				���ش�����Ϣ
	* postscript																													*/
#define IST8310_WHO_AM_I 0x00       //ist8310 who am I �Ĵ���
#define IST8310_WHO_AM_I_VALUE 0x10 //�豸 ID
#define IST8310_WRITE_REG_NUM 4 //IST8310��Ҫ���õļĴ�����Ŀ
static const	uint8_t ist8310_write_reg_data_error[IST8310_WRITE_REG_NUM][3]={
		//�Ĵ�����ַ	д���ֵ	���صĴ�����
	{			0x0B,			0x08,			0x01		},			//�����жϣ������õ͵�ƽ
	{			0x41,			0x09,			0x02		},			//ƽ����������
	{			0x42,			0xC0,			0x03		},			//����Ĵ����������ó�0xC0
	{			0x0A,			0x0B,			0x04		},			//200Hz���Ƶ��
};
uint8_t ist8310_init(void)
{
    static const uint8_t wait_time = 1;
    static const uint8_t sleepTime = 50;
    uint8_t res = 0;
    uint8_t writeNum = 0;
		/*(1) ��ʼ��GPIO��ͨ��*/
    ist8310_GPIO_init();
    ist8310_com_init();// �����պ�������Ϊhal���ʼ���Ѿ����ã����ʹ��ģ��I2C�����Լ�д
		/*(2) �����豸*/
    ist8310_RST_L(); // ��ƽ���͸�λ����
    ist8310_delay_ms(sleepTime);// ��ʱ50ms������
    ist8310_RST_H(); // ������
    ist8310_delay_ms(sleepTime);// ����ʱ50ms
		/*(3) ��֤�豸ID*/
    res = ist8310_IIC_read_single_reg(IST8310_WHO_AM_I);// ��who_am_i�Ĵ��������������ֵ����res
    if (res != IST8310_WHO_AM_I_VALUE)
    {
        return IST8310_NO_SENSOR;
    }
    ist8310_delay_ms(wait_time);
    /*(4) ����IST8310*/
    for (writeNum = 0; writeNum < IST8310_WRITE_REG_NUM; writeNum++)
    {
				/*(4.1) ��Ĵ���дֵ*/
        ist8310_IIC_write_single_reg(ist8310_write_reg_data_error[writeNum][0], ist8310_write_reg_data_error[writeNum][1]);
        ist8310_delay_ms(wait_time);
				/*(4.2) �ж�д��ȥ�ĶԲ���*/
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
  * brief			��ȡ�ų�����	
  *	param[out]		�ų�����
  * postscript																													*/
#define MAG_SEN 0.3f //ת���� uT
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
 * brief			����Ѿ�ͨ��I2C��DMA��ʽ��ȡ���˴�STA1��DATAZL�����ݣ�����ʹ������������д���
 * param[in]		����ָ�룬��STA1(0x02)�Ĵ��� ��DATAZL(0x08)�Ĵ���
 * param[out]		ist8310�����ݽṹ
 * postscript											*/
void ist8310_read_over(uint8_t *status_buf, ist8310_real_data_t *ist8310_real_data)
{

    if (status_buf[0] & 0x01)// ͨ���ж�STAT1�Ĵ�����λ�ж���û���µ����ݲ���
    {
        int16_t temp_ist8310_data = 0;
				/*(1) �޸�״̬λ*/
        ist8310_real_data->status |= 1 << IST8310_DATA_READY_BIT;
				/*(2) ����ƴ��*/
        temp_ist8310_data = (int16_t)((status_buf[2] << 8) | status_buf[1]);
        ist8310_real_data->mag[0] = MAG_SEN * temp_ist8310_data;
        temp_ist8310_data = (int16_t)((status_buf[4] << 8) | status_buf[3]);
        ist8310_real_data->mag[1] = MAG_SEN * temp_ist8310_data;
        temp_ist8310_data = (int16_t)((status_buf[6] << 8) | status_buf[5]);
        ist8310_real_data->mag[2] = MAG_SEN * temp_ist8310_data;
    }
    else// ����״̬λ����
    {
        ist8310_real_data->status &= ~(1 << IST8310_DATA_READY_BIT);
    }
}
