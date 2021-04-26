#include "flash.h"
#include "stm32f30x.h"
#include "protocal.h"
extern uint16_t save_flash[3];

uint16_t flash_read(uint32_t flash_address)
{
	int FLASHStatus;
	uint16_t flash_data=0;

	FLASH_Unlock();
	FLASHStatus = 1;
	FLASH_ClearFlag(FLASH_FLAG_BSY|FLASH_FLAG_EOP|FLASH_FLAG_PGERR);
	flash_data = *(uint16_t*) flash_address;
	FLASH_ClearFlag(FLASH_FLAG_BSY|FLASH_FLAG_EOP|FLASH_FLAG_PGERR);	
	FLASH_Lock();
	return flash_data;
}

void flash_write(uint32_t flash_address,uint16_t flash_data[],uint8_t data_length)
{
	int i=0,FLASHStatus;
	uint16_t *p = &flash_data[0];
	FLASH_Unlock();
	FLASHStatus = 1;
	FLASH_ClearFlag(FLASH_FLAG_BSY|FLASH_FLAG_EOP|FLASH_FLAG_PGERR);
	FLASHStatus = FLASH_ErasePage(flash_address);
	if(FLASHStatus == FLASH_COMPLETE) 
	{ 
		FLASHStatus = 1; //清空状态指示标志位
		for(i=0;i < data_length;i++)
			FLASHStatus = FLASH_ProgramHalfWord((flash_address+i*0x00000002),*(p+i));
	} 	
	FLASHStatus = 1; //清空状态指示标志位
	FLASH_ClearFlag(FLASH_FLAG_BSY|FLASH_FLAG_EOP|FLASH_FLAG_PGERR);	
	FLASH_Lock();
}

//void flash_set(uint32_t flash_address)
//{
//	if(flash_read(parameters_flash_address)==0xFFFF)
//		flash_write(parameters_flash_address,&save_flash[0]);
//}
