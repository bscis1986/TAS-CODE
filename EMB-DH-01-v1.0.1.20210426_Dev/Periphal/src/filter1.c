#include "filter.h"
#include "stdint.h"
#include "math.h"
extern uint8_t full_flag;
uint32_t  filter(uint8_t q_num, uint16_t (*data)[64][64], uint8_t line_n, uint8_t colum_n, uint8_t mag_f, float rc_f)
{
		uint8_t z=0;
		uint32_t psum=0;
	
		//*(*(*(data+q_num)+line_n)+colum_n) = *(*(*(data+q_num)+line_n)+colum_n)<<4;	//multi 16

		if(q_num>0)
		{	  
//			if(*(*(*(data+q_num)+line_n)+colum_n) >= *(*(*(data+q_num-1)+line_n)+colum_n))
//			{
//				*(*(*(data+q_num)+line_n)+colum_n) = (*(*(*(data+q_num)+line_n)+colum_n) - *(*(*(data+q_num-1)+line_n)+colum_n)) * rc_f + *(*(*(data+q_num-1)+line_n)+colum_n);
//			}
//			else
//			{
//				*(*(*(data+q_num)+line_n)+colum_n) =  *(*(*(data+q_num-1)+line_n)+colum_n) - (*(*(*(data+q_num-1)+line_n)+colum_n) - *(*(*(data+q_num)+line_n)+colum_n)) * rc_f ;
//			}
			
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
			psum = psum * 0.06667f;    // divide 15 ---4    31 -----5   0.03226f
		else
			psum = psum / (pow(2,(q_num + 1)) - 1);;
		
		//if(q_num==3)
		if(q_num == (QUEUE_LENGTH - 1))
		{
			for(z=0;z<(QUEUE_LENGTH - 1);z++)  //move queue
			{							
				*(*(*(data+z)+line_n)+colum_n) = *(*(*(data+z+1)+line_n)+colum_n);
			}
		}
		return psum;	
}

