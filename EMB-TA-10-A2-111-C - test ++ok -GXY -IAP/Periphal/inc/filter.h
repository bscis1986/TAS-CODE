#ifndef __FILTER_H
#define    __FILTER_H

#include "stdint.h"
#define QUEUE_LENGTH         5
#define MAX_LINE_NUM         64
#define MAX_COLUMN_NUM       64
#define TOTAL_GROUP          2
uint32_t  filter(uint8_t q_num, uint16_t (*data)[64][64], uint8_t line_n, uint8_t colum_n, uint8_t mag_f, float rc_f);
#endif
