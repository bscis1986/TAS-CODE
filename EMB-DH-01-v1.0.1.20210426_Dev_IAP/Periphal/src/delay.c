#include "delay.h"
#include "stm32f30x.h"
void Delay_time(uint32_t D_time)
{
	int  i=0,j=0;
	for(i=0;i<D_time;i++)
		for(j=0;j<5;j++);
}
