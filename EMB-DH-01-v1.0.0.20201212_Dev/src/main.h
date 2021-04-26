#ifndef __MAIN_H
#define __MAIN_H

#define Second_Row {GPIOE->BSRR  =  GPIO_Pin_2;GPIOE->BSRR =  GPIO_Pin_3;}
#define First_Row  {GPIOE->BRR  =  GPIO_Pin_2;GPIOE->BRR  =  GPIO_Pin_3;}
#include "stdint.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void Delay_time(uint32_t D_time);
void Blink_Led();
#endif /* __MAIN_H */