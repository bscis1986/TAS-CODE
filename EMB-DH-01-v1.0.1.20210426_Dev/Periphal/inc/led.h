#ifndef __LED_H
#define __LED_H
#define LED1_ON   GPIOE->BSRR  =  GPIO_Pin_1;
#define LED1_OFF  GPIOE->BRR  =   GPIO_Pin_1;
#define LED2_ON   GPIOE->BSRR  =  GPIO_Pin_0;
#define LED2_OFF  GPIOE->BRR  =   GPIO_Pin_0;
void blink_led(void);
#endif
