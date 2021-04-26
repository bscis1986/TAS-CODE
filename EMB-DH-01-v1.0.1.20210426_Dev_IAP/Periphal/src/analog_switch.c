#include "analog_switch.h"

uint8_t line_channel=1,line_num = 0;

void drive_switch_init(void)
{
	S0_L;
	S1_L;
	S2_L;
	S3_L;
	AS_U7_DISABLE;
	AS_U2_DISABLE;
	AS_U1_DISABLE;
	AS_U8_DISABLE;
}

void line_switch(void)
{	
	
	//disable all IC
	GPIOD->BRR  =  0x000F;
	
	if(line_channel == 1)
	{
		//change the channel of each IC
		GPIOB->BRR  =  0x0078; //0x0000
	}
	else
	{
		GPIOB->BRR   =  0x0078;   // set each port to low
		GPIOB->BSRR  =  0x0008 * (line_channel - 1);  // set the port to high
	}
		
	if(line_num < 16)
	{
		GPIOD->BSRR  =   0x0001;
	}
	else
	if(line_num > 15 && line_num < 32)
	{
		if(line_num == 16)
			line_channel = 1;
		GPIOD->BRR   =   0x0001;
		GPIOD->BSRR  =   0x0001 << 1;
	}
	else
	if(line_num > 31 && line_num < 48)
	{
		if(line_num == 32)
			line_channel = 1;
		GPIOD->BRR   =    0x0001 << 1;
		GPIOD->BSRR  =   0x0001 << 2;
	}
	else
	{
		if(line_num == 48)
			line_channel = 1;
		GPIOD->BRR   =   0x0001 << 2;
		GPIOD->BSRR  =   0x0001 << 3;
	}
	
	line_num++;
	line_channel++;
}
