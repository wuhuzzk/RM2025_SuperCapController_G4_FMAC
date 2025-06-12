#include "status_tip.h"
#include "Control_buck_boost.h"
#include "usart.h"
#include "uart_data_handle.h"
uint8_t led_state[4]={0};
uint16_t led_cnt[4]={0};

void status_flash (void)
{
	/* LED_0 */
if (led_state[0]==0)
{
	if (led_cnt[0]==0)
	{
		HAL_UART_Receive_IT(&huart3, &aRxBuffer, 1);
		HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_0);
		led_cnt[0]=30;
	}
	else led_cnt[0]--;
}
	/* LED_1 */
if (led_state[1]==1)
{
	if (led_cnt[1]==0)//等间隔闪，can正常
	{
		HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_1);
		led_cnt[1]=40;
		led_state[1]=0;
	}
	else led_cnt[1]--;
}
if (led_state[1]==2)//一秒两闪
{
	if (led_cnt[1]==100)HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,0);
	else if (led_cnt[1]==94)HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,1);
	else if (led_cnt[1]==84)HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,0);
	else if (led_cnt[1]==78)HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,1);
	else if (led_cnt[1]==0)
	{
		// HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_1);
		led_cnt[1]=101;
		led_state[1]=0;
	}
	led_cnt[1]--;
}
if (led_state[1]==3)//一秒三闪
{
	if (led_cnt[1]==300)HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,0);
	else if (led_cnt[1]==195)HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,1);
	else if (led_cnt[1]==180)HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,0);
	else if (led_cnt[1]==175)HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,1);
	else if (led_cnt[1]==160)HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,0);
	else if (led_cnt[1]==155)HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,1);
	else if (led_cnt[1]==0)
	{
		// HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_1);
		led_cnt[1]=305;
		led_state[1]=0;
	}
	led_cnt[1]--;
}

/* BUZZ */


if (Cap_Data.state_core==0xE2)
{
	if (led_cnt[3]==100)HAL_GPIO_WritePin(BUZZ_GPIO_Port,BUZZ_Pin,1);
	else if (led_cnt[3]==90)HAL_GPIO_WritePin(BUZZ_GPIO_Port,BUZZ_Pin,0);
	else if (led_cnt[3]==60)HAL_GPIO_WritePin(BUZZ_GPIO_Port,BUZZ_Pin,1);
	else if (led_cnt[3]==50)HAL_GPIO_WritePin(BUZZ_GPIO_Port,BUZZ_Pin,0);
	else if (led_cnt[3]==0)
	{
		led_cnt[3]=101;
		// led_state[3]=0;
	}
	led_cnt[3]--;
}
else if (Cap_Data.state_core==0xE1)
HAL_GPIO_WritePin(BUZZ_GPIO_Port,BUZZ_Pin,1);
// else if (Cap_Data.state_core==0||Cap_Data.state_core==0xEE||Cap_Data.state_core==0xA1)
// HAL_GPIO_WritePin(BUZZ_GPIO_Port,BUZZ_Pin,0);
}




