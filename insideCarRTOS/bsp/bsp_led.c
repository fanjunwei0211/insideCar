#include "bsp_led.h"
#include "main.h"
extern TIM_HandleTypeDef htim5;
/**
  * @brief          aRGB show
  * @param[in]      aRGB: 0xaaRRGGBB, 'aa' is alpha, 'RR' is red, 'GG' is green, 'BB' is blue
  * @retval         none
  */
/**
  * @brief          显示RGB
  * @param[in]      aRGB:0xaaRRGGBB,'aa' 是透明度,'RR'是红色,'GG'是绿色,'BB'是蓝色
  * @retval         none
  */
//void aRGB_led_show(uint32_t aRGB)
//{
//    static uint8_t alpha;
//    static uint16_t red,green,blue;

//    alpha = (aRGB & 0xFF000000) >> 24;
//    red = ((aRGB & 0x00FF0000) >> 16) * alpha;
//    green = ((aRGB & 0x0000FF00) >> 8) * alpha;
//    blue = ((aRGB & 0x000000FF) >> 0) * alpha;

//    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, blue);
//    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, green);
//    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, red);
//}

void led_flash(int cnt)
{
	static int i=0;
	static char t=0;
	i++;
	if(i>=cnt)
	{
		i=0;
		if(t==0)
		{
			HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,1);
			HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin,0);
			HAL_GPIO_WritePin(LED_B_GPIO_Port,LED_B_Pin,0);
		}			
		else if(t==1)
		{
			HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,0);
			HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin,1);
			HAL_GPIO_WritePin(LED_B_GPIO_Port,LED_B_Pin,0);
		}
		else
		{
			HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,0);
			HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin,0);
			HAL_GPIO_WritePin(LED_B_GPIO_Port,LED_B_Pin,1);
		}
		t++;
		if(t>2)
			t=0;
	}
}

//按键检测
int keyState()
{
	if(HAL_GPIO_ReadPin(KEY_GPIO_Port,KEY_Pin) == GPIO_PIN_RESET)
	{
		HAL_Delay(20);
		if(HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET)
		{
			while(HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET);
			return 1;
		}
	}
	return 0;
}
