#ifndef __FLASH_H
#define __FLASH_H

#include "stm32f30x.h"
void flash_set(uint32_t flash_address);
void flash_write(uint32_t flash_address,uint16_t flash_data[]);
uint16_t flash_read(uint32_t flash_address);
#endif
