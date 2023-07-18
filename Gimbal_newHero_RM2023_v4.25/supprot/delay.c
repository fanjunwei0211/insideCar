#include "delay.h"
#include "main.h"

/**
  * @brief          use cycle count to wait a time
  * @param[in]      us: us microseconds
  * @retval         none
  */
/**
  * @brief          ʹ��ѭ�������ӳ�һ��ʱ��
  * @param[in]      us:us΢��
  * @retval         none
  */
void user_delay_us(uint16_t us)
{
    for(; us > 0; us--)
    {
        for(uint8_t i = 50; i > 0; i--)
        {
            ;
        }
    }
}

/**
  * @brief          use user_delay_us function to wait a time
  * @param[in]      ms: ms milliseconds
  * @retval         none
  */
/**
  * @brief          ʹ��user_delay_us�����ӳ�һ��ʱ��
  * @param[in]      ms:ms����
  * @retval         none
  */
void user_delay_ms(uint16_t ms)
{
    for(; ms > 0; ms--)
    {
        user_delay_us(1000);
    }
}
