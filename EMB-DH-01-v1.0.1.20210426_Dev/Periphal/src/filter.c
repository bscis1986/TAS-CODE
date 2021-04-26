#include "filter.h"
#include "stdint.h"
#include "math.h"
extern uint8_t full_flag;

static uint32_t power_function(uint8_t n)
{
    uint32_t power_result = 1;
		uint8_t i = 0;
		for(i=1;i<= n;i++)
		power_result *= 2;	
		return power_result;
}

uint32_t  filter(uint8_t q_num, uint16_t (*data)[64][64], uint8_t line_n, uint8_t colum_n, uint8_t mag_f, float rc_f)
{
		uint8_t z=0;
		uint32_t psum=0;
	
		if(q_num>0)
		{	  
			*(*(*(data+q_num)+line_n)+colum_n) = *(*(*(data+q_num)+line_n)+colum_n)*rc_f + *(*(*(data+q_num-1)+line_n)+colum_n)*(1-rc_f);
		}
		else	
		{			
			*(*(*(data+q_num)+line_n)+colum_n) = *(*(*(data+q_num)+line_n)+colum_n);
		}

	
		for(z=0;z<(q_num+1);z++)
		{
			psum = psum + (*(*(*(data+z)+line_n)+colum_n)<<z);
		}
		
		if(full_flag)
		{
			psum = psum * 0.06667f;    // divide 15 ---4    31 -----5   0.03226f
		}
		else
		{
		    psum = psum / (power_function(q_num + 1) - 1);
		}

		if(q_num == (QUEUE_LENGTH - 1))
		{
			for(z=0;z<(QUEUE_LENGTH - 1);z++)  //move queue
			{							
				*(*(*(data+z)+line_n)+colum_n) = *(*(*(data+z+1)+line_n)+colum_n);
			}
		}
		return psum;	
}

