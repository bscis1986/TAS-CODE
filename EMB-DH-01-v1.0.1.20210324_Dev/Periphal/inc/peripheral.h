#ifndef __PERIPHERAL_H
#define __PERIPHERAL_H
#include "stdint.h"
#define START_TIMER TIM_Cmd(TIM3,ENABLE)
#define STOP_TIMER  TIM_Cmd(TIM3,DISABLE)
#define LED_SET_BIT          frame_data.frame_buffer[4]
#define FLASH_PARAMETERS     (uint8_t *) (0x08008800)

void TIM3_Config(uint16_t time_set);
void DAC_Config(void);
void TIM3_NVIC_Config(void);
void ADC_DMA_Configuration(void);
void RCC_Config(void);
void GPIO_Config(void);
void peripheral_init(void);
void RTC_Config(void);
void RTC_RESET(void);
void led_set(void);
void reset_rtc(void);
void send_buf_init(void);
#endif
