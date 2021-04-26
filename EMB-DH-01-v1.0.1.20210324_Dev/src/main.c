#include "hw_config.h"
#include "protocal.h"
#include "peripheral.h"
#include "analog_switch.h"
#include "stdarg.h"
#include <stdio.h> 
#include <math.h> 
#include <string.h> 
#include "main.h"
#include <arm_math.h>
#include "protocal.h"
#include "dac.h"
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_pwr.h"
#include "filter.h"
#include "parameter_common.h"
#include "stm32f30x_rtc.h"
#include "led.h"

#define PACKAGE_STYLE        frame_data.frame_buffer[3]
#define FILTER_ON            *(uint16_t*) (parameters_flash_address + 2)   // set the refresh to flash , read the parameter

extern uint8_t receive_buffer[32],copy_flag;;
extern uint16_t Receive_length,amp_factor_get;
extern uint8_t Line_Over_Flag,start_work,sample_time,line_channel,line_num;
extern send_buf_union send_buf_un;
extern uint32_t time_total;
extern parameter_get_str parameter_get;
extern float large_pressure_factor;
uint16_t value_final[QUEUE_LENGTH][MAX_LINE_NUM][MAX_COLUMN_NUM]={0};
uint32_t packet_sent=1,psum=0,packet_receive=1;
uint8_t  full_flag=0;

int main(void)
{
	uint8_t i=0,j=0; 
	uint16_t (*p)[64][64] = value_final; 
	float R_Factor = 0;
	peripheral_init();
  while (1)
 	{					 	
    if(receive_buffer[0] == FRAME_HEAD && copy_flag == 1)
		{
				copy_flag = 0;
				receive_buffer[0] = 0;
				EscapeRecoverFunction(receive_buffer,frame_data.frame_buffer,Receive_length);
				switch(PACKAGE_STYLE)
				{
						case PARA_SET_COMMAND:			
								 parameters_set();
								 break;
						case LINK_SET_COMMAND:
								 software_link();
								 //start the pc software send link command and timestamp command
								 if(receive_buffer[18] == TIMESTAMP_SET_COMMAND)
								 {
										 receive_buffer[18] = 0;
										 reset_rtc();
								 }
								 break;
						case LED_SET_COMMAND:
								 led_set();
						break;
						case TIMESTAMP_SET_COMMAND:
								 reset_rtc();
						break;
						default:
								 break;
				}
		}			
    if(Line_Over_Flag == 1)   //finish one line scan
		{
		    Line_Over_Flag = 0;				
				if(line_num == (MAX_LINE_NUM + 1))
				{
				    drive_switch_init();
						STOP_TIMER;
						//R_Factor = (float)39.49f * parameter_get.dac_set_get * P_AMP_FACTOR * large_pressure_factor;
					  R_Factor = (float)0.0079f * amp_factor_get * parameter_get.dac_set_get * P_AMP_FACTOR ;
						if(FILTER_ON != SAMPLE_RATE)  //filter function off
						{				
						    sample_time = QUEUE_LENGTH - 1;
								for(i = 0;i < MAX_LINE_NUM; i++)
								    for(j = 0;j < MAX_COLUMN_NUM; j++)
										{						
										    send_buf_un.send_buf.filter_data[i][j] = *(*(*(p + sample_time)+i)+j) * R_Factor;
												if(send_buf_un.send_buf.filter_data[i][j] < parameter_get.min_threshold_Value)
												{
												    send_buf_un.send_buf.filter_data[i][j] = 0;   
												}
										}
						}
						else
						{
						    for(i = 0;i < MAX_LINE_NUM; i++)   //filter function on
								    for(j = 0;j < MAX_COLUMN_NUM; j++)
										{
										    value_final[sample_time][i][j] = filter(sample_time,value_final,i,j, 0, FLITER_FACTOR);
												send_buf_un.send_buf.filter_data[i][j] = (float) value_final[sample_time][i][j] * R_Factor;
												if(send_buf_un.send_buf.filter_data[i][j] < parameter_get.min_threshold_Value)
												{
																send_buf_un.send_buf.filter_data[i][j] = 0;   
												}
										}
								if(sample_time >= QUEUE_LENGTH - 1)
								{
										sample_time = QUEUE_LENGTH - 1;  //the queue is full 
										full_flag = 1;				       
								}
								else
								{
										sample_time++;		              // the queue is not full	
								}						
						}			
						line_num = 0;
						line_channel = 1;
						process_data();
						trans_data();			
						blink_led();					
						START_TIMER;			
				}	
		}
	}
}
#ifdef USE_FULL_ASSERT
/*******************************************************************************
* Function Name  : assert_failed
* Description    : Reports the name of the source file and the source line number
*                  where the assert_param error has occurred.
* Input          : - file: pointer to the source file name
*                  - line: assert_param error line source number
* Output         : None
* Return         : None
*******************************************************************************/
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {}
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
