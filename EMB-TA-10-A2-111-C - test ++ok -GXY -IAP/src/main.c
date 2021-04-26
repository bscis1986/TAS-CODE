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
#include "crypto.h"
#include "Encrypt.h"
#include "led.h"
#include "flash.h"

//#define PACKAGE_STYLE        frame_data.frame_buffer[3]
//#define FILTER_ON            *(uint16_t*) 0x0803f802   // set the refresh to flash , read the parameter
//#define RC_FILTER_FACTOR     0.32f
//#define SAMPLE_RATE          10
//#define DEBUG                0

//extern uint8_t receive_buffer[64];
//extern uint16_t Receive_length ;
//extern uint8_t line_over_flag,start_work,sample_time,line_channel,line_num;
//extern send_buf_union send_buf_un;
//extern uint32_t time_total;
extern uint8_t Plaintext[PLAINTEXT_LENGTH];
extern uint8_t Key[CRL_AES128_KEY];
extern uint8_t IV[CRL_AES_BLOCK];
//extern parameter_get_str parameter_get;
extern uint8_t OutputMessage[PLAINTEXT_LENGTH];
extern uint32_t OutputMessageLength ;
//uint16_t value_final[QUEUE_LENGTH][MAX_LINE_NUM][MAX_COLUMN_NUM]={0};
uint32_t packet_sent=1,psum=0;//packet_receive=1;
//uint8_t  full_flag=0;

int main(void)
{
	uint32_t mcu_id[3]={0},status; 
	const uint32_t rand_num[6] = {0x9C5E05F4,0x02C76AF3,0x40D29CD9,0x83a1f258,0x1d389d43,0x5f2d6b6D};	
	uint8_t i=0;
	uint16_t output_data[9]={0};
	uint16_t write_data=0x0001;
	
	Crypto_DeInit(); 
	
	//get aes key
	for(i=0;i<4;i++)
	{
		Key[i]    =  ((*(volatile uint32_t *)(rand_num[0]^rand_num[3]) ^ rand_num[2]))>>(i*8) & 0xFF;
		Key[i+4]  =  ((*(volatile uint32_t *)(rand_num[1]^rand_num[4]) ^ rand_num[5]))>>(i*8) & 0xFF;
		Key[i+8]  =  ((*(volatile uint32_t *)(rand_num[2]^rand_num[5]) ^ rand_num[1]))>>(i*8) & 0xFF;
		Key[i+12] =  ((0x80A4B117 ^ 0x55667788)) >> (i*8) & 0xFF;
	}
	/*encrypt*/
	status = STM32_AES_CTR_Encrypt((uint8_t *) Plaintext, PLAINTEXT_LENGTH, Key, IV, sizeof(IV), OutputMessage,&OutputMessageLength);
                            
	for(i=0;i<8;i++)
		output_data[i] = OutputMessage[i*2] | OutputMessage[i*2+1]<<8;
	output_data[8] = 0x0001;
	flash_write(0x08010000,&output_data[0],9);//parameters flash location
	
	//flash_write(0x8010200,&write_data,1);//flag
	
	while (1)
	{		
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
