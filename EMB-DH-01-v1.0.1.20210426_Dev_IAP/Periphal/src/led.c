#include "led.h"
#include "stm32f30x.h"
void blink_led()
{
		static uint8_t led_flag=0;
		if(led_flag==1)
		{
			LED1_ON;
			led_flag = 0;
		}
		else
		{
			LED1_OFF;	
			led_flag = 1;
		}
}
