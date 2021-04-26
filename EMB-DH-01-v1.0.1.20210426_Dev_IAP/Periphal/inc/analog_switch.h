#ifndef __ANALOG_SWITCH_H
#define __ANALOG_SWITCH_H

#include "stm32f30x.h"

#define SECOND_GROUP   {GPIOB->BSRR  =  GPIO_Pin_10;GPIOB->BSRR =  GPIO_Pin_11;}
#define FIRST_GROUP    {GPIOB->BRR   =  GPIO_Pin_10;GPIOB->BRR  =  GPIO_Pin_11;}
//#define SECOND_GROUP         {GPIOE->BSRR  =  GPIO_Pin_2;GPIOE->BSRR =  GPIO_Pin_3;}
//#define FIRST_GROUP          {GPIOE->BRR  =  GPIO_Pin_2;GPIOE->BRR  =  GPIO_Pin_3;}
#define AS_U7_DISABLE            GPIOD->BRR  =  GPIO_Pin_3;//EN4
#define AS_U7_ENABLE             GPIOD->BSRR  =  GPIO_Pin_3;
#define AS_U2_DISABLE         GPIOD->BRR  =   GPIO_Pin_0;
#define AS_U2_ENABLE   GPIOD->BSRR  =  GPIO_Pin_0;//EN1
#define AS_U1_DISABLE  GPIOD->BRR  =  GPIO_Pin_1;//EN2
#define AS_U1_ENABLE   GPIOD->BSRR  =  GPIO_Pin_1;
#define AS_U8_DISABLE  GPIOD->BRR  =  GPIO_Pin_2;//EN3
#define AS_U8_ENABLE   GPIOD->BSRR  =  GPIO_Pin_2;

#define AS_CH_HIGH     GPIOB->BSRR;
#define AS_CH_LOW      GPIOB->BRR;

#define S0_H  GPIOB->BSRR  =  GPIO_Pin_3;
#define S0_L  GPIOB->BRR   =  GPIO_Pin_3;

#define S1_H  GPIOB->BSRR  =  GPIO_Pin_4;
#define S1_L  GPIOB->BRR   =  GPIO_Pin_4;

#define S2_H  GPIOB->BSRR  =  GPIO_Pin_5;
#define S2_L  GPIOB->BRR   =  GPIO_Pin_5;

#define S3_H  GPIOB->BSRR  =  GPIO_Pin_6;
#define S3_L  GPIOB->BRR   =  GPIO_Pin_6;

//#define Drive_Switch_H  GPIOC->BSRR =  GPIO_Pin_6;
//#define Drive_Switch_L  GPIOC->BRR  =  GPIO_Pin_6;

void drive_switch_init(void);
void Device_Config(void);
void Line_Switch_Config(void);
void line_switch(void);
#endif
