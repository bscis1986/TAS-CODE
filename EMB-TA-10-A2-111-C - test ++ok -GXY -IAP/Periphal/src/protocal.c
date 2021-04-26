#include "protocal.h"
#include "stm32f30x.h"
#include "hw_config.h"
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_pwr.h"
#include "flash.h"
#include "analog_switch.h"
#include "dac.h"
#include "filter.h"
#include "peripheral.h"
#include "parameter_common.h"
#include "stm32f30x_rtc.h"
#include <stdarg.h>

unsigned char  UserTxBufferFS[64];
#define  parameters_length    0x11//17

#define  cdc_package_size     64

uint8_t start_work=0;
uint16_t Length_byte=1,data_number=0;
extern uint16_t filter_data[MAX_LINE_NUM][MAX_COLUMN_NUM];
uint32_t check_sum=0;
uint8_t data_final[9000]={0},escape_data[64]={0};;
uint16_t save_flash[3]={10,20,10};

//+HEAD(1)+LENGTH(2)+COMMAND(1)+TIMESTAMP(4)+VERSION(4)+RAW DATA(8192) + CHECKSUM(4)

uint8_t time_h=0,time_m=0,time_s=0;
uint16_t time_mm=0;
uint32_t time_total=0;

extern uint8_t line_channel,line_num;
frame_union  frame_data ;
send_buf_union send_buf_un;
RTC_TimeTypeDef RTC_TimeStruct;
RTC_TimeTypeDef RTC_TimeStructure;
parameter_get_str parameter_get = {10,0.1f,20};

void receive_buffer_init(void)
{
	frame_data.parameter_data_send.parameter_data.package_head  = 0xAA;
	frame_data.parameter_data_send.parameter_data.package_length =0x000E;
	frame_data.parameter_data_send.parameter_data.command = 0x81;
	frame_data.parameter_data_send.parameter_data.gain_set = 5000;
	frame_data.parameter_data_send.parameter_data.min_threshold_value = parameter_get.min_threshold_Value;
	frame_data.parameter_data_send.parameter_data.line_num = 0x40;
	frame_data.parameter_data_send.parameter_data.column_num = 0x40;
	frame_data.parameter_data_send.parameter_data.refresh_rate = 0x0000;
	frame_data.parameter_data_send.parameter_data.dac_voltage_set = parameter_get.dac_set_get;
	frame_data.parameter_data_send.version_data.hardware_version = HV;
	frame_data.parameter_data_send.version_data.firmware_version = FV;
	frame_data.parameter_data_send.check_sum = 0x0160;
}

uint16_t EscapeFunction(uint8_t data[],uint8_t escapedata[],uint16_t datalength,uint16_t start_index)
{
		uint16_t i=0,escape_data_length=1;
	  for(i=start_index;i < datalength;i++)
		{		
			if(FRAME_HEAD != *(data+i) || 0x55 != *(data+i) )
			{
				 *(escapedata + escape_data_length) = *(data + i);
				 escape_data_length++;			
			}									
			else
		  {
			  *(escapedata + escape_data_length) = 0x55;				
				*(escapedata + escape_data_length+1) = (*(data + i) ^ 0x20);				
				escape_data_length += 2;
		  }
		}	
		return escape_data_length;
}

void EscapeRecoverFunction(uint8_t data[],uint8_t escapedata[],uint16_t datalength)
{
		uint8_t s=1,t=1;
	  while(t < datalength)
		{
			if(0x55 == data[t])
			{
				//Get_Buffer[jj] = 0x55;
				escapedata[s] =( data[t+1] ^ 0x20);
				t = t+2;
			}
			else
			{
				escapedata[s] = data[t];
				++t;
			}
			++s;
		}
}

void get_timestamp(void)
{
	RTC_GetTime(RTC_Format_BIN,&RTC_TimeStruct);
	time_total = (RTC_TimeStruct.RTC_Hours *3600 + RTC_TimeStruct.RTC_Minutes *60 + RTC_TimeStruct.RTC_Seconds) * 1000 + ((2000 - RTC_GetSubSecond())>>1);
}

void process_data(void)
{
	get_timestamp();
	send_buf_un.send_buf.time_stamp = time_total;
	Length_byte = EscapeFunction(send_buf_un.final_data,data_final,8207,0);//8719
}

void trans_delay(uint16_t d_time)
{
	uint16_t i=0;
	for(i=0;i<d_time;i++);
}

void trans_data(void)
{
	uint8_t i=0,cdc_package_num=0;
	//cdc_package_num = Length_byte / cdc_package_size;
	cdc_package_num = Length_byte >> 6;
  data_final[0] = FRAME_HEAD;
	for(i=0; i < cdc_package_num;i++)
	{
		while(GetEPTxStatus(ENDP1) != EP_TX_NAK);
		CDC_Send_DATA((uint8_t*)(data_final + i*cdc_package_size),cdc_package_size);
	}
	while(GetEPTxStatus(ENDP1) != EP_TX_NAK);
	CDC_Send_DATA((uint8_t*)(data_final + cdc_package_num*cdc_package_size),(Length_byte % cdc_package_size));
	Length_byte = 1;
	data_number = 0;
}

void cdc_send_function(uint8_t data[],uint8_t data_length)
{
	while(GetEPTxStatus(ENDP1)!= EP_TX_NAK);
	CDC_Send_DATA((uint8_t*)data,data_length);
}

//void parameters_set(void)
//{
//	static uint8_t escape_data_length=0;
// 
//	save_flash[0] = frame_data.parameter_data_receive.parameter_data.min_threshold_value;
//	save_flash[1] = frame_data.parameter_data_receive.parameter_data.refresh_rate; //filter on/off   
//	save_flash[2] = frame_data.parameter_data_receive.parameter_data.dac_voltage_set;
//	
//	flash_write(parameters_flash_address,&save_flash[0]);
//	
//	parameter_get.min_threshold_Value = frame_data.parameter_data_receive.parameter_data.min_threshold_value;
//	parameter_get.dac_set_get = 1.0 /frame_data.parameter_data_receive.parameter_data.dac_voltage_set;
//	parameter_get.filter_state_get = frame_data.parameter_data_receive.parameter_data.refresh_rate;
//	
//	for(uint8_t i = 1;i < parameters_length;i++)
//		frame_data.parameter_data_send.check_sum += frame_data.frame_buffer[i];
//	
//	escape_data_length = EscapeFunction(frame_data.frame_buffer,escape_data,19,1);
//	escape_data[0] = FRAME_HEAD;
//	cdc_send_function(escape_data,escape_data_length);
//	
//	escape_data_length = 1;
//	if(frame_data.parameter_data_send.parameter_data.min_threshold_value == 0)
//		frame_data.parameter_data_send.parameter_data.min_threshold_value = 10;

//	drivesignal_set(1.5f);
//}

void software_link(void)
{
	uint8_t escape_data_length=0;
	static uint8_t system_on = 0;
	
	parameter_get.min_threshold_Value = *(uint16_t*) 0x0803f800;
	parameter_get.dac_set_get = *(uint16_t*) 0x0803f804;
	
  receive_buffer_init();
	escape_data_length= EscapeFunction(frame_data.frame_buffer,escape_data,19,1);
	escape_data[0] = FRAME_HEAD;
	cdc_send_function(escape_data,escape_data_length);
	escape_data_length = 1;
  if(frame_data.parameter_data_send.parameter_data.min_threshold_value == 0)
			frame_data.parameter_data_send.parameter_data.min_threshold_value = 10;		
	if(!system_on)
	{
		drivesignal_set(1.5f);
		system_on = 1;	
		drive_switch_init();
		line_channel = 1;
		line_num=0;
		START_TIMER;
	}
}

void usb_printf(const char *format, ...)
{
    va_list args;
    uint32_t length;
 
    va_start(args, format);
    length = vsnprintf((char *)UserTxBufferFS, 64, (char *)format, args);
    va_end(args);
    CDC_Send_DATA((uint8_t*)(UserTxBufferFS),length);
}
