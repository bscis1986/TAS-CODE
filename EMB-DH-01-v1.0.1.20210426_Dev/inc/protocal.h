#ifndef __PROTOCAL_H
#define __PROTOCAL_H
#include "stdint.h"
#pragma anon_unions

#define  parameters_flash_address  		0x08010800
#define  FRAME_HEAD            			  0xAA
#define  DRIVE_VOLTAGE_SET     			  1.5f
#define  LINE_COUNT 		      			  frame_data.parameter_data_receive.parameter_data.line_num
#define  COLUMN_COUNT 	      			  frame_data.parameter_data_receive.parameter_data.column_num
#define  LINE_SET      		  			    64
#define  COLUMN_SET            			  64
#define  LOCATION_SIZE 		  			    512
#define  HV                   			  0x0001
#define  FV                   			  0x0101
#define  matrix_data_command  			  0x7F
#define  debug                        0
typedef __packed struct
{
	uint16_t min_threshold_Value;
	float  dac_set_get;
	uint16_t filter_state_get;
}parameter_get_str;

typedef __packed struct
{
	uint16_t hardware_version;
	uint16_t firmware_version;
}version_data_struct;

typedef __packed enum
{
	DATA_SEND_COMMAND  = 0x79,	
	PARA_SET_COMMAND   = 0x80,
	LINK_SET_COMMAND   = 0x81,
	LED_SET_COMMAND    = 0x82,
	TIMESTAMP_SET_COMMAND = 0x83,
}command_set;

typedef __packed struct
{
	uint8_t     package_head;//1
	uint16_t    package_length;//2
	command_set command;//1
	uint16_t    gain_set;//2
	uint16_t    min_threshold_value;//2
	uint8_t  		line_num;//1
	uint8_t 	  column_num;//1
	uint16_t 		refresh_rate;//2
	uint8_t  		dac_voltage_set;//1
}parameter_data_struct;

typedef __packed struct
{
	parameter_data_struct parameter_data; //13
	version_data_struct   version_data;  //4
	uint16_t              check_sum;	  //2
}parameter_data_send_struct;

typedef __packed struct
{
	parameter_data_struct    parameter_data; //13
	uint16_t                 check_sum;	 //2
}parameter_data_receive_struct;

typedef __packed union
{
	uint8_t frame_buffer[32];
	__packed union
	{
		parameter_data_send_struct      parameter_data_send;	//19
		parameter_data_receive_struct   parameter_data_receive;  //15
	};
}frame_union;

typedef __packed struct
{
	uint16_t send_length;  //2
	uint8_t send_command;  //1
	uint32_t time_stamp;   //4
	uint16_t hard_ver;     //2
	uint16_t firm_ver;      //2
	uint16_t filter_data[64][64];  //8192
	uint32_t send_data_checksum;   //4
}send_buf_struct;
typedef __packed union
{
	send_buf_struct   send_buf;
	uint8_t  final_data[8207];//8719
}send_buf_union;



extern frame_union  frame_data;

void process_data(void);
void parameters_set(void);
//uint8_t *set_loction(uint16_t data[][64]);
uint16_t EscapeFunction(uint8_t data[],uint8_t escapedata[],uint16_t datalength,uint16_t start_index);
void EscapeRecoverFunction(uint8_t data[],uint8_t escapedata[],uint16_t datalength);
void software_link(void);
void trans_data(void);
void trans_data1(void);
void receive_buffer_init(void);
//void usb_printf(const char *format, ...);
#endif


