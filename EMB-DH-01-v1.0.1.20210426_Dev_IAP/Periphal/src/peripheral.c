#include "peripheral.h"
#include "stdarg.h"
#include "protocal.h"
#include "hw_config.h"
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_pwr.h"
#include "delay.h"
#include "flash.h"
#include "analog_switch.h"
#include "crypto.h"
#include "Encrypt.h"
#include "parameter_common.h"
#include "stm32f30x_rtc.h"
#include "led.h"
#define ADC1_DR_ADDRESS ((uint32_t)0x50000040)
#define ADC2_DR_ADDRESS ((uint32_t)0x50000140)
#define ADC3_DR_ADDRESS ((uint32_t)0x50000440)
#define ADC4_DR_ADDRESS ((uint32_t)0x50000540)
#define ADCS_Sampletime ADC_SampleTime_7Cycles5
#define ApplicationAddress   0x08020000
#define ID_ADDRESS            0x1FFFF7AC


uint16_t ADC1ConvertedValue[9]={0};
uint16_t ADC2ConvertedValue[6]={0};
uint16_t ADC3ConvertedValue[10]={0};
uint16_t ADC4ConvertedValue[7]={0};

uint32_t mcuID[4] = {0};
extern uint8_t Plaintext[PLAINTEXT_LENGTH];
extern uint8_t Key[16];
extern uint8_t IV[16];
extern uint32_t OutputMessageLength,time_total;
extern send_buf_union send_buf_un;
DAC_InitTypeDef            DAC_InitStructure;
RCC_ClocksTypeDef          RCC_Clocks;
RTC_InitTypeDef   RTC_InitStructure;

void TIM3_Config(uint16_t time_set)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;/* 定义用于初始化的结构体 */
  //意思是采用内部定时器来作为分频器使用的
  TIM_InternalClockConfig(TIM3);/* 配置定时器3的内部时钟源(启用) */
  //TIM_DeInit( TIM3);
  /* 定时器基本配置 */
  TIM_TimeBaseStructure.TIM_Period = (time_set-1);
  TIM_TimeBaseStructure.TIM_Prescaler = (72-1);
  
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//clock division 1
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);
  
  /* 清除溢出中断标志 */
  TIM_ClearFlag(TIM3,TIM_FLAG_Update);
  /* 禁止ARR预装载缓冲器 */
  TIM_ARRPreloadConfig(TIM3,DISABLE);
  /* 开启TIM3的中断 */
  TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
  
 // TIM_Cmd(TIM3,ENABLE);/* 使能TIM3计数器功能 */
}

void DAC_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_DAC, ENABLE);
  RCC_APB1PeriphResetCmd(RCC_APB1Periph_DAC, DISABLE);
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
  DAC_InitStructure.DAC_Trigger = DAC_Trigger_None;
  DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
  //DAC_InitStructure.DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bits11_0;
  DAC_InitStructure.DAC_Buffer_Switch = DAC_BufferSwitch_Disable;
  DAC_Init(DAC1,DAC_Channel_1,&DAC_InitStructure);

  DAC_Cmd(DAC1,DAC_Channel_1,ENABLE);
	
//	DAC_SetChannel1Data(DAC1,DAC_Align_12b_R, 3/3.3*4096);
//	DAC_SoftwareTriggerCmd(DAC1,DAC_Channel_1,ENABLE);
}


void TIM3_NVIC_Config(void)
{
	NVIC_InitTypeDef  NVIC_TIM3_InitStructure;
  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);/* 使能TIM3定时器的时钟 */
  
  /* Enable the RTC Alarm Interrupt */
  NVIC_TIM3_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_TIM3_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; //1
  NVIC_TIM3_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_TIM3_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_TIM3_InitStructure);
  
  /* 清除相关中断标志位 */
	//TIM_ClearFlag(TIM3,TIM_FLAG_Update);
  TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
}

void ADC_DMA_Configuration(void)
{
  uint32_t calibration_value1=0,calibration_value2=0,calibration_value3=0,calibration_value4=0;
	DMA_InitTypeDef DMA_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	GPIO_InitTypeDef      GPIO_InitStructure;
//	//初始化
	DMA_DeInit(DMA1_Channel1);   //ADC1
	DMA_DeInit(DMA2_Channel1);   //ADC2
  DMA_DeInit(DMA2_Channel5);   //ADC3
	DMA_DeInit(DMA2_Channel2);   //ADC4
	/*   ADC1    */
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC1_DR_ADDRESS;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&ADC1ConvertedValue;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = 9;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;	
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
  DMA_Cmd(DMA1_Channel1, ENABLE);
	/*   ADC2   */
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC2_DR_ADDRESS;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&ADC2ConvertedValue;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = 6;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA2_Channel1, &DMA_InitStructure);
  DMA_Cmd(DMA2_Channel1, ENABLE);
	/*   ADC3   */
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC3_DR_ADDRESS;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&ADC3ConvertedValue;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = 10;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA2_Channel5, &DMA_InitStructure);
  DMA_Cmd(DMA2_Channel5, ENABLE);

	/*    ADC4   */
	
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC4_DR_ADDRESS;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&ADC4ConvertedValue;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = 7;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA2_Channel2, &DMA_InitStructure);
  DMA_Cmd(DMA2_Channel2, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_5| GPIO_Pin_6| GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_12| GPIO_Pin_14| GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4| GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11| GPIO_Pin_12| GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOF, &GPIO_InitStructure);
	
	 /* Calibration procedure */ 
	 ADC_VoltageRegulatorCmd(ADC1, ENABLE);
	 ADC_VoltageRegulatorCmd(ADC2, ENABLE);
	 ADC_VoltageRegulatorCmd(ADC3, ENABLE);
	 ADC_VoltageRegulatorCmd(ADC4, ENABLE);
	 
	// Delay_time(20000);
	 
	 ADC_SelectCalibrationMode(ADC1, ADC_CalibrationMode_Single);
	 ADC_StartCalibration(ADC1);
	 while(ADC_GetCalibrationStatus(ADC1) != RESET );
	 calibration_value1 = ADC_GetCalibrationValue(ADC1);
	 
	 ADC_SelectCalibrationMode(ADC2, ADC_CalibrationMode_Single);
	 ADC_StartCalibration(ADC2);
	 while(ADC_GetCalibrationStatus(ADC2) != RESET );
	 calibration_value2 = ADC_GetCalibrationValue(ADC2);
	 
	 ADC_SelectCalibrationMode(ADC3, ADC_CalibrationMode_Single);
	 ADC_StartCalibration(ADC3);
	 while(ADC_GetCalibrationStatus(ADC3) != RESET );
	 calibration_value3 = ADC_GetCalibrationValue(ADC3);
	 
	 ADC_SelectCalibrationMode(ADC4, ADC_CalibrationMode_Single);
	 ADC_StartCalibration(ADC4);
	 while(ADC_GetCalibrationStatus(ADC4) != RESET );
	 calibration_value4 = ADC_GetCalibrationValue(ADC4);

	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Clock = ADC_Clock_SynClkModeDiv2;  
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStructure.ADC_DMAMode = ADC_DMAMode_Circular;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay =0x02;
	
  ADC_CommonInit(ADC1, &ADC_CommonInitStructure);
	ADC_CommonInit(ADC2, &ADC_CommonInitStructure);
	ADC_CommonInit(ADC3, &ADC_CommonInitStructure);
	ADC_CommonInit(ADC4, &ADC_CommonInitStructure);
	
	ADC_InitStructure.ADC_ContinuousConvMode = ADC_ContinuousConvMode_Disable;
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b; 
  //ADC_InitStructure.ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_0;         
  ADC_InitStructure.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_OverrunMode = ADC_OverrunMode_Disable;   
  ADC_InitStructure.ADC_AutoInjMode = ADC_AutoInjec_Disable;   
  ADC_InitStructure.ADC_NbrOfRegChannel = 9;
  ADC_Init(ADC1, &ADC_InitStructure);
	
	ADC_InitStructure.ADC_ContinuousConvMode = ADC_ContinuousConvMode_Disable;
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b; 
  //ADC_InitStructure.ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_0;         
  ADC_InitStructure.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_OverrunMode = ADC_OverrunMode_Disable;   
  ADC_InitStructure.ADC_AutoInjMode = ADC_AutoInjec_Disable;   
  ADC_InitStructure.ADC_NbrOfRegChannel = 6;
  ADC_Init(ADC2, &ADC_InitStructure);
	
	ADC_InitStructure.ADC_ContinuousConvMode = ADC_ContinuousConvMode_Disable;
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b; 
  //ADC_InitStructure.ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_0;         
  ADC_InitStructure.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_OverrunMode = ADC_OverrunMode_Disable;   
  ADC_InitStructure.ADC_AutoInjMode = ADC_AutoInjec_Disable;   
  ADC_InitStructure.ADC_NbrOfRegChannel = 10;
  ADC_Init(ADC3, &ADC_InitStructure);
	
	ADC_InitStructure.ADC_ContinuousConvMode = ADC_ContinuousConvMode_Disable;
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b; 
  //ADC_InitStructure.ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_0;         
  ADC_InitStructure.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_OverrunMode = ADC_OverrunMode_Disable;   
  ADC_InitStructure.ADC_AutoInjMode = ADC_AutoInjec_Disable;   
  ADC_InitStructure.ADC_NbrOfRegChannel = 7;
  ADC_Init(ADC4, &ADC_InitStructure);
	
	ADC_RegularChannelConfig(ADC1, ADC_Channel_6,1, ADCS_Sampletime);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_7,2, ADCS_Sampletime);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_8,3, ADCS_Sampletime);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_9,4, ADCS_Sampletime);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_10,5, ADCS_Sampletime);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_1,6, ADCS_Sampletime);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_2,7, ADCS_Sampletime);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_3,8, ADCS_Sampletime);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4,9, ADCS_Sampletime);
	
	ADC_RegularChannelConfig(ADC2, ADC_Channel_2, 1, ADCS_Sampletime);
  ADC_RegularChannelConfig(ADC2, ADC_Channel_3, 2, ADCS_Sampletime);
  ADC_RegularChannelConfig(ADC2, ADC_Channel_4, 3, ADCS_Sampletime);
  ADC_RegularChannelConfig(ADC2, ADC_Channel_5, 4, ADCS_Sampletime);
  ADC_RegularChannelConfig(ADC2, ADC_Channel_11, 5, ADCS_Sampletime);
  ADC_RegularChannelConfig(ADC2, ADC_Channel_12,6, ADCS_Sampletime);
  //ADC_RegularChannelConfig(ADC2, ADC_Channel_14,7, ADCS_Sampletime);
	
	ADC_RegularChannelConfig(ADC3, ADC_Channel_7, 1, ADCS_Sampletime);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_12, 2, ADCS_Sampletime);
  ADC_RegularChannelConfig(ADC3, ADC_Channel_1,3, ADCS_Sampletime);
  ADC_RegularChannelConfig(ADC3, ADC_Channel_13,4, ADCS_Sampletime);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_6,5, ADCS_Sampletime);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_2,6, ADCS_Sampletime);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_14,7, ADCS_Sampletime);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_15,8, ADCS_Sampletime);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_16,9, ADCS_Sampletime);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_3,10, ADCS_Sampletime);
	
	ADC_RegularChannelConfig(ADC4, ADC_Channel_1,1, ADCS_Sampletime);
  ADC_RegularChannelConfig(ADC4, ADC_Channel_2,2, ADCS_Sampletime);
  ADC_RegularChannelConfig(ADC4, ADC_Channel_3,3, ADCS_Sampletime);
	ADC_RegularChannelConfig(ADC4, ADC_Channel_13,4, ADCS_Sampletime);
  ADC_RegularChannelConfig(ADC4, ADC_Channel_4,5, ADCS_Sampletime);
  ADC_RegularChannelConfig(ADC4, ADC_Channel_5,6, ADCS_Sampletime);
	ADC_RegularChannelConfig(ADC4, ADC_Channel_12,7, ADCS_Sampletime);
	 
  ADC_Cmd(ADC1, ENABLE);
  ADC_Cmd(ADC2, ENABLE);
	ADC_Cmd(ADC3, ENABLE);
	ADC_Cmd(ADC4, ENABLE);
	

	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_RDY));
	while(!ADC_GetFlagStatus(ADC2, ADC_FLAG_RDY));
	while(!ADC_GetFlagStatus(ADC3, ADC_FLAG_RDY));
	while(!ADC_GetFlagStatus(ADC4, ADC_FLAG_RDY));
	
	ADC_DMACmd(ADC1, ENABLE);
  ADC_DMACmd(ADC2, ENABLE);
  ADC_DMACmd(ADC3, ENABLE);
	ADC_DMACmd(ADC4, ENABLE);
	
	ADC_DMAConfig(ADC1, ADC_DMAMode_Circular);
	ADC_DMAConfig(ADC2, ADC_DMAMode_Circular);
	ADC_DMAConfig(ADC3, ADC_DMAMode_Circular);
	ADC_DMAConfig(ADC4, ADC_DMAMode_Circular);
	
}

void RCC_Config(void)
{
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA,  ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB,  ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC,  ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOD,  ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOE,  ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOF,  ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC,ENABLE);
	
	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ADC12, ENABLE);	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ADC34, ENABLE);
	RCC_ADCCLKConfig(RCC_ADC12PLLCLK_Div2);
	RCC_ADCCLKConfig(RCC_ADC34PLLCLK_Div2);
}


void GPIO_Config(void)
{
	
	GPIO_InitTypeDef GPIO_InitStructure;
		/*   ADC  */
	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_7);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_7);
	
	//GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_7);    
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &GPIO_InitStructure); 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_10 | GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType= GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9| GPIO_Pin_12;
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType= GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_4|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_5|GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType= GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType= GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType= GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);	

//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//	GPIO_InitStructure.GPIO_OType= GPIO_OType_PP;
//	//GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOC, &GPIO_InitStructure);	
}

void Get_ID(void)
{
	uint8_t i=0;
	mcuID[0] =   *(volatile u32*)(ID_ADDRESS);
	mcuID[1] =   *(volatile u32*)(ID_ADDRESS+4);
	mcuID[2] =   *(volatile u32*)(ID_ADDRESS+8);
	
	mcuID[0] = mcuID[0]^0x12345678;
	mcuID[1] = mcuID[1]^0x87654321;
	mcuID[2] = mcuID[2]^0x11223344;
	mcuID[3] = 0x80A4B117^0x55667788;
	
	for(i=0;i<4;i++) 
	{
		Key[i] = (mcuID[0]>>(i*8)) & 0xFF;
		Key[i+4] = (mcuID[1]>>(i*8)) & 0xFF;
		Key[i+8] = (mcuID[2]>>(i*8)) & 0xFF;
		Key[i+12] = (mcuID[3]>>(i*8)) & 0xFF;		
	}
}

void Decrypt_Function(void)
{
	int32_t status = AES_SUCCESS;
	
	status = STM32_AES_CTR_Decrypt( (uint8_t *) (0x08008800), PLAINTEXT_LENGTH, Key, IV, sizeof(IV), (uint8_t*)(0x2000ff00),
                            &OutputMessageLength);
  if (status == AES_SUCCESS)
  {
		if(*((uint16_t*)(0x2000ff00)) == 0x50)
		{
			GPIOE->BSRR  =  GPIO_Pin_1;
 	    //while(1);
		}
  }
}

uint32_t Flash_EnableReadProtection(void)
{
  /* Returns the FLASH Read Protection level. */
  if( FLASH_OB_GetRDP() == RESET )
  {
   // GPIOE->BSRR  =  GPIO_Pin_1;
		FLASH_Unlock();
		/* Unlock the Option Bytes */
    FLASH_OB_Unlock();    
    /* Sets the read protection level. */
    FLASH_OB_RDPConfig(OB_RDP_Level_1);		
		//FLASH_OB_Launch();	
    FLASH_OB_Lock();
		FLASH_Lock();	
	}
}

void RTC_Config(void)
{
  uint32_t tmpreg1=0; 

	/* Enable the PWR clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

  /* Allow access to RTC */
  PWR_BackupAccessCmd(ENABLE);
  
  tmpreg1 = (RCC->BDCR & ~(RCC_BDCR_RTCSEL));
  
  RCC_BackupResetCmd(ENABLE);
  RCC_BackupResetCmd(DISABLE);
  
  RCC->BDCR = tmpreg1;
  
  //RTC_WaitForSynchro();
    
//#if defined (RTC_CLOCK_SOURCE_LSI)  /* LSI used as RTC source clock*/
  /* The RTC Clock may varies due to LSI frequency dispersion. */   
  /* Enable the LSI OSC */ 
 //--- RCC_LSICmd(ENABLE);

  /* Wait till LSI is ready */  
  while(RCC_GetFlagStatus(RCC_FLAG_HSERDY) == RESET)
  {
  }

  /* Select the RTC Clock Source */
 //--- RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);
	RCC_RTCCLKConfig(RCC_RTCCLKSource_HSE_Div32);
	RCC_RTCCLKCmd(ENABLE);
	
	//RTC_WaitForSynchro();
  
 // SynchPrediv = 0x7CF;  //0x18f
 // AsynchPrediv = 0x7c;  //0x27
	
	RTC_InitStructure.RTC_AsynchPrediv = 0x7c;
	RTC_InitStructure.RTC_SynchPrediv = 0x7CF;
	RTC_InitStructure.RTC_HourFormat = RTC_HourFormat_24;
	
	RTC_Init(&RTC_InitStructure);
}

void RTC_RESET(void)
{
	RCC_RTCCLKConfig(RCC_RTCCLKSource_HSE_Div32);
	RCC_RTCCLKCmd(ENABLE);
	
	RTC_InitStructure.RTC_AsynchPrediv = 0x7c;
	RTC_InitStructure.RTC_SynchPrediv = 0x7CF;
	RTC_InitStructure.RTC_HourFormat = RTC_HourFormat_24;
	
	RTC_Init(&RTC_InitStructure);
}

void send_buf_init(void)
{
	send_buf_un.send_buf.send_length = 8201;//8713
	send_buf_un.send_buf.send_command = matrix_data_command;//matrix_data_command;
	send_buf_un.send_buf.hard_ver = HV;
	send_buf_un.send_buf.firm_ver = FV;
	send_buf_un.send_buf.send_data_checksum = 0;
}

void led_set(void)
{
	if(LED_SET_BIT == 0x01)
	{
		LED2_ON;
	}
	else	
	{						
		LED2_OFF;
	}
}

void reset_rtc(void)
{
	RTC_Config();
	time_total = 0;
}

void jump_decryption_function(void)
{
	uint8_t i=0;
	uint32_t JumpAddress = 0,mcu_id[3]={0};
	unsigned char key_data[16]={0};//0x1FFFF7AC    1FFF F7B0   1FFF F7B4   
	const uint32_t rand_num[6] = {0x9C5E05F4,0x02C76AF3,0x40D29CD9,0x83a1f258,0x1d389d43,0x5f2d6b6D};
	typedef  void (*pFunction)(void);
	pFunction Jump_To_Application;
	/*jump to second part*/
	if(*(volatile uint32_t *)(0x803E400) == 0xFFFFFFFF)
	{		 
		//usart_disable(USART0);
		//dma_channel_disable(DMA0, DMA_CH4);
		JumpAddress = *(__IO uint32_t*) (ApplicationAddress + 4);
		Jump_To_Application = (pFunction) JumpAddress;
		__set_MSP(*(__IO uint32_t*) ApplicationAddress);
		Jump_To_Application();
	}
	/*Decrypt the parameters*/
	else
	{
//		//get uid
//		mcu_id[0] = *(volatile uint32_t *)(rand_num[0]^rand_num[3]);
//		mcu_id[1] = *(volatile uint32_t *)(rand_num[1]^rand_num[4]);
//		mcu_id[2] = *(volatile uint32_t *)(rand_num[2]^rand_num[5]);
		
		/*base on uid , get key*/
		for(i=0;i<4;i++)
		{
			key_data[i]    =  ((*(volatile uint32_t *)(rand_num[0]^rand_num[3]) ^ rand_num[2]))>>(i*8) & 0xFF;
			key_data[i+4]  =  ((*(volatile uint32_t *)(rand_num[1]^rand_num[4]) ^ rand_num[5]))>>(i*8) & 0xFF;
			key_data[i+8]  =  ((*(volatile uint32_t *)(rand_num[2]^rand_num[5]) ^ rand_num[1]))>>(i*8) & 0xFF;
			key_data[i+12] =  ((0x80A4B117 ^ 0x55667788)) >> (i*8) & 0xFF;
		}
		/*decryption*/
		Decrypt_Function();
		
		/*earse the second part code*/
		
		
//		mbedtls_aes_init(&aes);
//		mbedtls_aes_setkey_dec(&aes,key_data,128);
//		mbedtls_aes_crypt_ecb(&aes,MBEDTLS_AES_DECRYPT,(uint8_t*)(0x08019c00),(uint8_t*)(0x2000Bf00)); 
		//fmc_erase_pages();	// erase the second part	

//		fmc_unlock();
//    /* clear all pending flags */
//    fmc_flag_clear(FMC_FLAG_BANK0_END);
//    fmc_flag_clear(FMC_FLAG_BANK0_WPERR);
//    fmc_flag_clear(FMC_FLAG_BANK0_PGERR);    
//    /* erase the flash pages */
//		fmc_page_erase(0x803E400);
//		//fmc_word_program(0x803E400, 0x01);
//		//*(volatile uint32_t *)(0x803E400) = 0x01020304;
//		fmc_flag_clear(FMC_FLAG_BANK0_END);
//		fmc_flag_clear(FMC_FLAG_BANK0_WPERR);
//		fmc_flag_clear(FMC_FLAG_BANK0_PGERR);
//    /* lock the main FMC after the erase operation */
//    fmc_lock();			
	}
}

void peripheral_init(void)
{
	RCC_Config();
	GPIO_Config();
  Set_System();
  Set_USBClock();
	DAC_Config();
	ADC_DMA_Configuration();
  USB_Interrupts_Config();
  USB_Init();
	drive_switch_init();
	Flash_EnableReadProtection();
	Crypto_DeInit(); 
	Get_ID();
	Decrypt_Function();
	TIM3_NVIC_Config();
	TIM3_Config(60);       //set 100us
	//flash_set(parameters_flash_address); // store parameters to flash
	RTC_Config();
	send_buf_init();
}
