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

//unsigned char  UserTxBufferFS[64];
#define  parameters_length    0x11//17

#define  cdc_package_size     64

uint8_t start_work=0;
uint16_t Length_byte=1,data_number=0;
extern uint16_t filter_data[MAX_LINE_NUM][MAX_COLUMN_NUM];
uint32_t check_sum=0;
uint8_t data_final[8500]={0},escape_data[32]={0};;
uint16_t save_flash[3]={10,20,10};
float large_pressure_factor = 1;
uint16_t amp_factor_get = 5000;

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
	frame_data.parameter_data_send.parameter_data.refresh_rate = parameter_get.filter_state_get;
	frame_data.parameter_data_send.parameter_data.dac_voltage_set = 10;
	frame_data.parameter_data_send.version_data.hardware_version = HV;
	frame_data.parameter_data_send.version_data.firmware_version = FV;
	frame_data.parameter_data_send.check_sum = 0x0160;
}

uint16_t EscapeFunction(uint8_t data[],uint8_t escapedata[],uint16_t datalength,uint16_t start_index)
{
		uint16_t i=0,escape_data_length=1;
	  for(i=start_index;i < datalength;i++)
		{		
			if(FRAME_HEAD == *(data+i) || 0x55 == *(data+i) )
			{
				 *(escapedata + escape_data_length) = 0x55;				
				*(escapedata + escape_data_length+1) = (*(data + i) ^ 0x20);				
				escape_data_length += 2;
			}									
			else
		  {			 			
				*(escapedata + escape_data_length) = *(data + i);
				 escape_data_length++;	
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
	send_buf_init();
	Length_byte = EscapeFunction(send_buf_un.final_data,data_final,8207,0);//8719
}

//void trans_data(void)
//{
//	uint8_t i=0,cdc_package_num=0;
//	cdc_package_num = Length_byte >> 6;
//	data_final[0] = FRAME_HEAD;
//	for(i=0; i < cdc_package_num;i++)
//	{
//		while(GetEPTxStatus(ENDP1) != EP_TX_NAK);                 //usb发送是否完成
//		CDC_Send_DATA((uint8_t*)(data_final + i * cdc_package_size),cdc_package_size);
//	}
//	while(GetEPTxStatus(ENDP1) != EP_TX_NAK);
//	CDC_Send_DATA((uint8_t*)(data_final + cdc_package_num*cdc_package_size),(Length_byte % cdc_package_size));
//	Length_byte = 1;
//	data_number = 0;
//}

void trans_data1(void)
{
		
	uint16_t Send_length=0,p_size=0,i=0,trans_time=0,less_length=0,full_time = 0,pack_flag = 0,len = 0,usb_in_data_remain = 0,usb_in_numofpackage=0,plus_data=0;
 // uint8_t *p = data_final;
  //	less_length = Length_byte%64;
  //Length_byte = 150;
	usb_in_numofpackage = Length_byte / 64 + 1;
	data_final[0] = FRAME_HEAD;
	usb_in_data_remain = Length_byte;
	//往两个缓存存入最初的64bytes数据
////////////////////////////
  UserToPMABufferCopy(data_final, ENDP1_TXADDR, 64);
  while(GetEPTxStatus(ENDP1)!= EP_TX_NAK);
  SetEPDblBuf1Count(ENDP1, EP_DBUF_IN, 64);
	
  UserToPMABufferCopy(data_final, ENDP1_TXADDR1, 64);
  while(GetEPTxStatus(ENDP1)!= EP_TX_NAK);
  SetEPDblBuf0Count(ENDP1, EP_DBUF_IN, 64);		 
///////////////////////////
	
	usb_in_data_remain -= 64;
	plus_data += 64;
/*
  首先在1、2两块缓存区，放入最开始的 64 Bytes数据。
	第一次进入，交换缓存区，系统控制1，用户控制2，系统马上发送1，然后往缓存区2发放数据。
	第二次进入，交换缓存区，系统控制2，用户控制1，系统发送2，然后往1存数据。
	第三次进入，交换缓存区，系统控制1，用户控制2，系统发送1，然后往2存数据。

*/
	while(usb_in_numofpackage != 0)
	{
	    usb_in_numofpackage --;   
		 
		 if(GetENDPOINT(ENDP1) & EP_DTOG_RX) 
		 {
		     if(usb_in_numofpackage >= 0)
				 {
				     FreeUserBuffer(ENDP1, EP_DBUF_IN); //释放缓存区占用
				 }
				
			
			if(usb_in_data_remain > 0)
			{
			 if(usb_in_data_remain > VIRTUAL_COM_PORT_DATA_SIZE)
			 {
						len = VIRTUAL_COM_PORT_DATA_SIZE;
			 }
			 else
			 {
						len = usb_in_data_remain;
			 }
			 SetEPTxValid(ENDP1);//发送缓冲区数据
			 
			 UserToPMABufferCopy(data_final + plus_data, ENDP1_TXADDR, len); //往空闲的缓存区放数据
			 while(GetEPTxStatus(ENDP1)!= EP_TX_NAK);
			 SetEPDblBuf0Count(ENDP1, EP_DBUF_IN, len);
			 
			 //SetEPTxValid(ENDP1);//buf1发走
			 
			 usb_in_data_remain -= len;
			 plus_data += len;
			}
			else
			{
				//SetEPDblBuf0Count(ENDP1, EP_DBUF_IN, 0);
				SetEPTxValid(ENDP1);
			}
 }
 else
 {
			if(usb_in_numofpackage >= 0)
			{
					FreeUserBuffer(ENDP1, EP_DBUF_IN);
			}
			
			if(usb_in_data_remain > 0)
			{
			 if(usb_in_data_remain > VIRTUAL_COM_PORT_DATA_SIZE)
			 {				
					len = VIRTUAL_COM_PORT_DATA_SIZE;
			 }
			 else
			 {
					len = usb_in_data_remain;
			 }
			 
			 SetEPTxValid(ENDP1);
			 
			 UserToPMABufferCopy(data_final + plus_data, ENDP1_TXADDR1, len);
			 while(GetEPTxStatus(ENDP1)!= EP_TX_NAK);
			 SetEPDblBuf1Count(ENDP1, EP_DBUF_IN, len);	
			 
			// SetEPTxValid(ENDP1);
			 
			 usb_in_data_remain -= len;
			 plus_data += len;
			}
			else
			{
			 // SetEPDblBuf1Count(ENDP1, EP_DBUF_IN, 0);
				SetEPTxValid(ENDP1);
			}
 }
 }				
	Length_byte = 1;
	data_number = 0;
	Send_length = 0;
}

void cdc_send_function(uint8_t data[],uint8_t data_length)
{
	while(GetEPTxStatus(ENDP1)!= EP_TX_NAK);
	CDC_Send_DATA((uint8_t*)data,data_length);
}

void parameters_set(void)
{
	static uint8_t escape_data_length=0;
//	float test_5v =0;
 
	save_flash[0] = frame_data.parameter_data_receive.parameter_data.min_threshold_value;
	save_flash[1] = frame_data.parameter_data_receive.parameter_data.refresh_rate; //filter on/off   
	save_flash[2] = frame_data.parameter_data_receive.parameter_data.dac_voltage_set;
	
	amp_factor_get = frame_data.parameter_data_receive.parameter_data.gain_set;
	
//	if(save_flash[2] == 0x01)
//	{
//		large_pressure_factor = 0.5f;
//	}
//	else
//	{
//		large_pressure_factor = 1;
//	}
	
	flash_write(parameters_flash_address,&save_flash[0]);
	
	parameter_get.min_threshold_Value = frame_data.parameter_data_receive.parameter_data.min_threshold_value;
	parameter_get.dac_set_get = (float)1.0f /frame_data.parameter_data_receive.parameter_data.dac_voltage_set;
	parameter_get.filter_state_get = frame_data.parameter_data_receive.parameter_data.refresh_rate;
	
	for(uint8_t i = 1;i < parameters_length;i++)
	{
		frame_data.parameter_data_send.check_sum += frame_data.frame_buffer[i];
	}
	escape_data_length = EscapeFunction(frame_data.frame_buffer,escape_data,19,1);
	escape_data[0] = FRAME_HEAD;
	cdc_send_function(escape_data,escape_data_length);
	
	escape_data_length = 1;
	if(frame_data.parameter_data_send.parameter_data.min_threshold_value == 0)
	{
		frame_data.parameter_data_send.parameter_data.min_threshold_value = 10;
	}
	drivesignal_set(P_DRIVESIGNAL);
	send_buf_init();
}

void software_link(void)
{
	uint8_t escape_data_length=0;
	static uint8_t system_on = 0;
	
	parameter_get.min_threshold_Value = *(uint16_t*) parameters_flash_address;
	parameter_get.dac_set_get         = *(uint16_t*)(parameters_flash_address + 4);
	parameter_get.dac_set_get = (float) 1.0f /frame_data.parameter_data_receive.parameter_data.dac_voltage_set;
  receive_buffer_init();
	escape_data_length = EscapeFunction(frame_data.frame_buffer,escape_data,19,1);
	escape_data[0] = FRAME_HEAD;
	cdc_send_function(escape_data,escape_data_length);
	escape_data_length = 1;
  if(frame_data.parameter_data_send.parameter_data.min_threshold_value == 0)
	{
		frame_data.parameter_data_send.parameter_data.min_threshold_value = 10;
	}
  send_buf_init();	
	if(system_on == 0)
	{
		drivesignal_set(P_DRIVESIGNAL);
		system_on = 1;	
		drive_switch_init();
		line_channel = 1;
		line_num=0;
		START_TIMER;
	}
}

void disable_nvic_usb(FunctionalState nvic_state)
{
    NVIC_InitTypeDef NVIC_InitStructure; 
  	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; //2  //1
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
}

//void usb_printf(const char *format, ...)
//{
//    va_list args;
//    uint32_t length;
// 
//    va_start(args, format);
//    length = vsnprintf((char *)UserTxBufferFS, 64, (char *)format, args);
//    va_end(args);
//    CDC_Send_DATA((uint8_t*)(UserTxBufferFS),length);
//}
