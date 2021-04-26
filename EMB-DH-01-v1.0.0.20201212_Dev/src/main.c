#include "hw_config.h"
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_pwr.h"
#include "stdarg.h"
#include <stdio.h> 
#include <math.h> 
#include <string.h> 
#include "main.h"
#include <arm_math.h>
#ifdef _GNUC_
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
	#else
#define PUTCHAR_PROTOTYPE int fputc(int ch,FILE *f)
	#endif

DAC_InitTypeDef            DAC_InitStructure;

#define ADC1_DR_ADDRESS ((uint32_t)0x50000040)
#define ADC2_DR_ADDRESS ((uint32_t)0x50000140)
#define ADC3_DR_ADDRESS ((uint32_t)0x50000440)
#define ADC4_DR_ADDRESS ((uint32_t)0x50000540)

#define ADCS_Sampletime ADC_SampleTime_7Cycles5

////////////////////////////
#define Drive_Switch_H  GPIOC->BSRR =  GPIO_Pin_6;
#define Drive_Switch_L  GPIOC->BRR  =  GPIO_Pin_6;
#define LED1_ON  GPIOE->BSRR  =  GPIO_Pin_1;
//#define LED2_ON  GPIOE->BSRR  =  GPIO_Pin_0;
#define LED1_OFF  GPIOE->BRR  =  GPIO_Pin_1;
//#define LED2_OFF  GPIOE->BRR  =  GPIO_Pin_0;
//#define U1_ENABLE  GPIOD->BRR  =  GPIO_Pin_7;
//#define U1_DISABLE  GPIOD->BSRR  =  GPIO_Pin_7;
//#define U2_ENABLE  GPIOD->BRR  =  GPIO_Pin_4;
//#define U2_DISABLE  GPIOD->BSRR  =  GPIO_Pin_4;
//#define U3_ENABLE  GPIOD->BRR  =  GPIO_Pin_5;
//#define U3_DISABLE  GPIOD->BSRR  =  GPIO_Pin_5;
//#define U4_ENABLE  GPIOD->BRR  =  GPIO_Pin_6;
//#define U4_DISABLE  GPIOD->BSRR  =  GPIO_Pin_6;

//U2,U1,U8,U7
#define U7_DISABLE  GPIOD->BRR  =  GPIO_Pin_3;//EN4
#define U7_ENABLE  GPIOD->BSRR  =  GPIO_Pin_3;
#define U2_DISABLE  GPIOD->BRR  =   GPIO_Pin_0;
#define U2_ENABLE   GPIOD->BSRR  =  GPIO_Pin_0;//EN1
#define U1_DISABLE  GPIOD->BRR  =  GPIO_Pin_1;//EN2
#define U1_ENABLE  GPIOD->BSRR  =  GPIO_Pin_1;
#define U8_DISABLE  GPIOD->BRR  =  GPIO_Pin_2;//EN3
#define U8_ENABLE  GPIOD->BSRR  =  GPIO_Pin_2;


#define S0_H  GPIOB->BSRR  =  GPIO_Pin_3;
#define S0_L  GPIOB->BRR   =  GPIO_Pin_3;

#define S1_H  GPIOB->BSRR  =  GPIO_Pin_4;
#define S1_L  GPIOB->BRR   =  GPIO_Pin_4;

#define S2_H  GPIOB->BSRR  =  GPIO_Pin_5;
#define S2_L  GPIOB->BRR   =  GPIO_Pin_5;

#define S3_H  GPIOB->BSRR  =  GPIO_Pin_6;
#define S3_L  GPIOB->BRR   =  GPIO_Pin_6;

//10K  0.722
//330K  200k 0.075
//51K 33k  0.622
//100k   0.422
//2.2k  0.782

///////////////////////////////
//#define VREF 0.822
float VIO=0;
#define drive_voltage_set 1.5
///////////////////////////////

////////////////////////////
extern __IO uint8_t Receive_Buffer[32];//64改为32
extern __IO  uint32_t Receive_length ;
extern __IO  uint32_t length ;
extern uint8_t Line_over_flag,adc_n;
uint8_t Send_Buffer[32],UpLoad_Buffer[32]={0},UpLoad_BufferTwo[32]={0};//64改为32
uint8_t Get_Buffer[32],Up_checksum=0;//64改为32
//uint16_t ADC1ConvertedValue[22]={0};
uint32_t packet_sent=1,check_sum=0,SumValue=0;
float AverageSum=0;
uint32_t packet_receive=1;
uint8_t data_final[13000]={0};//8500
uint16_t Line_Num = 0,Line_Channel=1,String_Len=0,StartWork=0,L1_ON=1,senttime=0,SampleTime=0,TransTime=0,TranSecTime=0,LL=0;
uint8_t FinalDate[13000]={0};
uint16_t sum1=0,calibration_value1=0,calibration_value2=0,calibration_value3=0,calibration_value4=0,data_number=0;
uint16_t zz=0,Round_flag=0,Sample_time=0,i3=0,flash_data=0,save_flash[3]={10,10,10};
int Length_byte=1,mm=0;
uint8_t Row_Colum_Num[512]={0};
uint16_t return_data=0;
//参数设置
uint8_t MaxLineNum=64,MaxRowNum=64;//尺寸
uint8_t Dac_Voltage_Set = 10;//激励信号
uint16_t MaxThresholdValue=10,MinThresholdValue=10;//门限值
uint16_t RefreshRate=0;
//
uint16_t zss=3,temp_exchange=0;
uint8_t CommandDate[32]={0},CommandFinalT[32]={0},CommandLength=0,CommandFinal[32]={0},CommandL=1,CommandLT=1;//64改为32

uint16_t ADC1ConvertedValue[9]={0};
uint16_t ADC2ConvertedValue[6]={0};
uint16_t ADC3ConvertedValue[10]={0};
uint16_t ADC4ConvertedValue[7]={0};

//uint16_t Value_buf_adc[6][64][64]={0};
uint16_t Value_buf_adc[8][64]={0};
uint16_t Value_Final[64][64]={0};

//float adjfactor[64]={0.99,0.98,0.99,0.98,0.99,0.99,0.995,0.99,1,0.98,0.99,0.98,1.005,0.985,0.985,0.985,
//								0.98,0.99,0.99,0.99,0.99,0.99,0.99,1,0.995,0.99,0.995,0.98,0.99,0.99,0.99,0.98,
//								0.995,0.99,0.98,0.98,0.97,0.99,0.98,0.99,0.99,0.99,1,0.993,0.995,0.995,0.98,0.99,
//								0.99,1,0.99,0.995,0.99,0.99,0.99,1,1,0.99,1,1,1,1,0.99,0.98,
//};

void Delay_time(uint32_t D_time)
{
	int i=0;
	for(i=0;i<D_time;i++);
}
void GPIO_Config()
{
	
	GPIO_InitTypeDef GPIO_InitStructure;
		/*   ADC  */

	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_7);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_7);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &GPIO_InitStructure); 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6;
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
				
}

PUTCHAR_PROTOTYPE
{
	while(!(CDC_Send_DATA((uint8_t *)&ch,1)) == 1) {};	
		return ch;
}

void Line_Switch(void)
{
	Line_Channel++;
	switch(Line_Num)
	{
		case 16:
		Line_Channel = 1;
		U2_DISABLE;
		U1_ENABLE;
		break;
		case 32:
		Line_Channel = 1;
		U1_DISABLE;
		U8_ENABLE;
		break;
		case 48:
		Line_Channel = 1;
		U8_DISABLE;
		U7_ENABLE;
		break;
	  case 64:
		Line_Channel = 1;
		U7_DISABLE;
		U2_DISABLE;
		U1_DISABLE;
		U8_DISABLE;
		break;
	}
			
	switch(Line_Channel)//choose the channle
	{
			case 1://0000
			S0_L;
			S1_L;
			S2_L;
			S3_L;		
			break;
			case 2://0001
			S0_H; 
			break;
			case 3://0010
			S0_L;
			S1_H;
			break;
			case 4://0011
			S0_H;
			S1_H;
			break;
			case 5://0100
			S0_L;
			S1_L;
			S2_H;
			break;
			case 6://0101
      S0_H;
			break;
			case 7://0110
			S0_L;
			S1_H;
      S2_H;	
			break;
			case 8://0111
			S1_H;
			S2_H;
			S0_H;	
			break;
			case 9://1000
			S3_H;
      S2_L;
      S1_L;
      S0_L;
			break;
			case 10://1001
			S0_H;
			break;
			case 11://1010
			S0_L;
      S1_H;
			break;
			case 12://1011		
			S0_H;	
			break;
			case 13://1100
			S2_H;
			S1_L;
			S0_L;
			break;
			case 14://1101
			S0_H;
			break;
			case 15://1110
			S1_H;
			S0_L;
			break;
			case 16://1111
			S0_H;
			break;				
		}
		//Drive_Switch_L;//三极管导通，使能驱动信号
}

void TIM3_Config(void)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;/* 定义用于初始化的结构体 */
  //意思是采用内部定时器来作为分频器使用的
  TIM_InternalClockConfig(TIM3);/* 配置定时器3的内部时钟源(启用) */
  //TIM_DeInit( TIM3);
  /* 定时器基本配置 */
	//16-1250
	//32-
	//4K-15.625,312 644 832
  TIM_TimeBaseStructure.TIM_Period = (80-1);//3000   624  660; /* 这里一定要减1 */
  TIM_TimeBaseStructure.TIM_Prescaler = (72-1); /* 到这里后就会将定时溢出设为1秒了，因为定时器时钟是60MHz */
  
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
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

void TIM3_NVIC_Config(void)
{
	NVIC_InitTypeDef  NVIC_TIM3_InitStructure;
  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);/* 使能TIM3定时器的时钟 */
  
  /* Enable the RTC Alarm Interrupt */
  NVIC_TIM3_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_TIM3_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_TIM3_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_TIM3_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_TIM3_InitStructure);
  
  /* 清除相关中断标志位 */
	//TIM_ClearFlag(TIM3,TIM_FLAG_Update);
  TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
}

void ADC_DMA_Configuration(void)
{

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
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_11 | GPIO_Pin_12| GPIO_Pin_14| GPIO_Pin_15;
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
	 
	 Delay_time(20000);
	 
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
	
//	ADC_StartCalibration(ADC1);
//	// 等待校准完成
//	while(ADC_GetCalibrationStatus(ADC1));
	
//	//ADC_VoltageRegulatorCmd(ADC1, ENABLE);
//  
//  /* Insert delay equal to 10 祍 */
// // Delay_time(100);
////  ADC_ResetCalibration(ADC1);
////  while(ADC_GetResetCalibrationStatus(ADC1));
//  
//  ADC_SelectCalibrationMode(ADC1, ADC_CalibrationMode_Single);
//  ADC_StartCalibration(ADC1);
//  
//  while(ADC_GetCalibrationStatus(ADC1) != RESET );
//  calibration_value = ADC_GetCalibrationValue(ADC1);
	
}

void RCC_Config()
{
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA,  ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB,  ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC,  ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOD,  ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOE,  ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOF,  ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ADC12, ENABLE);	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ADC34, ENABLE);
	RCC_ADCCLKConfig(RCC_ADC12PLLCLK_Div2);
	RCC_ADCCLKConfig(RCC_ADC34PLLCLK_Div2);
}

void DAC_Config()
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
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

void Blink_Led()
{
	  if(L1_ON==1)
		{
			LED1_ON;
			L1_ON = 0;
		}
		else
		{
		  LED1_OFF;	
			L1_ON = 1;
		}
}

void Line_Switch_Config()
{
	Drive_Switch_H;	
}

void Device_Config()
{
//	U1_DISABLE;
//	U3_DISABLE;
//	U4_DISABLE;
//	U2_DISABLE;
	U2_DISABLE;
	U7_DISABLE;
	U8_DISABLE;
	U1_DISABLE;
	Drive_Switch_H;//关激励信号
	S0_L;
	S1_L;
	S2_L;
	S3_L;
	Second_Row;
	//Second_Row;
	//First_Row;
	FinalDate[0] = 0xFF;
	FinalDate[1] = 0xFF;
}
void ProcessData()
{
	int ii=0,PackagLength=0;
	uint8_t i=0,j=0;	

///////////////////////////	坐标位置
	for(i=0;i<64;i++)
		for(j=0;j<64;j++)
		{
			if(Value_Final[i][j]>=MinThresholdValue)
				Row_Colum_Num[i*8+j/8]= ((0x01<<(j%8)) | Row_Colum_Num[i*8+j/8]);
			else
				Row_Colum_Num[i*8+j/8]= (0x00 | Row_Colum_Num[i*8+j/8]);				
		}
	for(mm=0;mm<512;mm++)
	{
		FinalDate[zss]= Row_Colum_Num[mm];
		if(Row_Colum_Num[mm]>0)
			data_number++;
		zss++;
	}
	
	for(mm=0;mm<512;mm++)
	{
		Row_Colum_Num[mm]= 0;
	}
//////////////////////////
	
	for(i=0;i<64;i++)
		for(j=0;j<64;j++)
		if(Value_Final[i][j] >= MinThresholdValue)//MinThresholdValue
		{
				FinalDate[zss] = (Value_Final[i][j])&0xFF;
				FinalDate[zss+1] = ((Value_Final[i][j])>>8)&0xFF;
				zss = zss+2;
		}
	PackagLength = zss - 2;//包含数据和命令字
	FinalDate[0] = PackagLength & 0xFF;
	FinalDate[1] = (PackagLength>>8) & 0xFF;
	FinalDate[2] = 0x7F;
	//for(i=0;i<zss;i++)
	//	check_sum += (uint32_t)FinalDate[i];
	check_sum = 0;
	FinalDate[zss] = check_sum & 0xFF;
	FinalDate[zss+1] = (check_sum>>8) & 0xFF;
	FinalDate[zss+2] = (check_sum>>16) & 0xFF;
	FinalDate[zss+3] = (check_sum>>24) & 0xFF;
  check_sum = 0;
	 for(ii=0;ii<(zss+4);ii++)
	{
		if(0xaa==FinalDate[ii] || 0x55==FinalDate[ii] )
		{
			data_final[Length_byte] = 0x55;
			data_final[Length_byte+1] =  (FinalDate[ii] ^ 0x20);	
			Length_byte=Length_byte+2;
		}									
		else
		{
			data_final[Length_byte] = FinalDate[ii];
			Length_byte++;
		}
	}
}

void TransData()
{
  //int i=0,j=0;	
	TransTime=Length_byte/63;
  TranSecTime=Length_byte%63;	
	zss=3;
  data_final[0] = 0xAA;
	if(Length_byte>8 && data_number>0)
	{
		for(LL=0;LL<TransTime;LL++)
		{
			while(GetEPTxStatus(ENDP1)!= EP_TX_NAK);
			CDC_Send_DATA((uint8_t*)(data_final+LL*63),63);
		}
		if(TranSecTime>0)
		{
			while(GetEPTxStatus(ENDP1)!= EP_TX_NAK);
			CDC_Send_DATA((uint8_t*)(data_final+TransTime*63),TranSecTime);
		}
	}
	Length_byte=1;
	data_number=0;
}
uint16_t flash_read(uint32_t flash_address)
{
	int i=0,FLASHStatus;
	uint16_t flash_data=0;
	//uint16_t *p = &flash_data[0];
	FLASH_Unlock();
	FLASHStatus = 1;
	FLASH_ClearFlag(FLASH_FLAG_BSY|FLASH_FLAG_EOP|FLASH_FLAG_PGERR);
	//FLASHStatus = FLASH_ErasePage(flash_address);
	//if(FLASHStatus == FLASH_COMPLETE) 
	//{ 
	//	FLASHStatus = 1; //清空状态指示标志位
		//for(i=0;i<3;i++)
		flash_data = *(uint16_t*) flash_address;
	//} 	
//	FLASHStatus = 1; //清空状态指示标志位
	FLASH_ClearFlag(FLASH_FLAG_BSY|FLASH_FLAG_EOP|FLASH_FLAG_PGERR);	
	//flash_data = *(uint16_t*) 0x0803f800;	
	FLASH_Lock();
	return flash_data;
}
void flash_write(uint32_t flash_address,uint16_t flash_data[])
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
		for(i=0;i<3;i++)
			FLASHStatus = FLASH_ProgramHalfWord((flash_address+i*0x00000002),*(p+i));
	} 	
	FLASHStatus = 1; //清空状态指示标志位
	FLASH_ClearFlag(FLASH_FLAG_BSY|FLASH_FLAG_EOP|FLASH_FLAG_PGERR);	
	//flash_data = *(uint16_t*) 0x0803f800;	
	FLASH_Lock();
}

int main(void)
 {
	unsigned char i=0,jj=0,ss=0,j=0;
	int sum_ubt=0;
	//uint16_t return_data=0;
	//uint32_t zj=0;
	//NVIC_InitTypeDef NVIC_InitStructure;
	RCC_ClocksTypeDef     RCC_Clocks;
	RCC_Config();
	GPIO_Config();
  Set_System();
  Set_USBClock();
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_DAC, ENABLE);
  RCC_APB1PeriphResetCmd(RCC_APB1Periph_DAC, DISABLE);
	DAC_Config();
	ADC_DMA_Configuration();
  USB_Interrupts_Config();
  USB_Init();
	Delay_time(10000);
	Device_Config();
	TIM3_NVIC_Config();
	TIM3_Config();
	Delay_time(1);//90us
	//return_data=flash_read();
	if(flash_read(0x0803f800)==0xffff)
		flash_write(0x0803f800,&save_flash[0]);
	RCC_GetClocksFreq(&RCC_Clocks);
  while (1)
  {				
		//return_data=(float)(75000000*10/1000*(1000*3.3f))/(5100*2.0*5*0.1*4096);	//150000000//59.24
		if(Receive_Buffer[0] == 0xAA)
		{
			jj=1;
			ss=0;
			Receive_Buffer[0] = 0;
			while(jj<Receive_length)
			{
				if(0x55 == Receive_Buffer[jj])
				{
					//Get_Buffer[jj] = 0x55;
					Get_Buffer[ss] =( Receive_Buffer[jj+1] ^ 0x20);
					jj = jj+2;
				}
				else
				{
					Get_Buffer[ss] = Receive_Buffer[jj];
					++jj;
				}
				++ss;
			}
			if(Get_Buffer[2] == 0x80)//set
			{
				MaxThresholdValue = (Get_Buffer[3] | (Get_Buffer[4]<<8));
				MinThresholdValue = (Get_Buffer[5] | (Get_Buffer[6]<<8 ));
				MaxLineNum = Get_Buffer[7];
				MaxRowNum =  Get_Buffer[8];
				RefreshRate = (Get_Buffer[9] | (Get_Buffer[10]<<8));
				Dac_Voltage_Set = Get_Buffer[11];		
				
				
				save_flash[0]=MinThresholdValue;
				save_flash[1]=MaxThresholdValue;
				save_flash[2]=Dac_Voltage_Set;
				
				flash_write(0x0803f800,&save_flash[0]);
				//write to flash
				/*
				FLASH_Unlock();
				FLASHStatus = 1;
				FLASH_ClearFlag(FLASH_FLAG_BSY|FLASH_FLAG_EOP|FLASH_FLAG_PGERR);
				FLASHStatus = FLASH_ErasePage(0x0803f800);
				
				//{
					if(FLASHStatus == FLASH_COMPLETE) 
					{ 
						FLASHStatus = 1; //清空状态指示标志位
						for(i=0;i<3;i++)
							FLASHStatus = FLASH_ProgramHalfWord(0x0803f800+i*0x00000002,save_flash[i]);
					} 
					FLASHStatus = 1; //清空状态指示标志位
				//}
				//FLASH_ProgramHalfWord(0x0803f000,0xabcd);
				//Delay_time(1000);
				//flash_data = *(uint16_t*) 0x0803f800;
				FLASH_ClearFlag(FLASH_FLAG_BSY|FLASH_FLAG_EOP|FLASH_FLAG_PGERR);
				FLASH_Lock();
				*/
				//end
			
				UpLoad_Buffer[0] = 0xAA;
			  UpLoad_Buffer[1] = 0x0E;
				UpLoad_Buffer[2] = 0;
				UpLoad_Buffer[3] = 0x80;
				for(i=4;i<13;i++)
					UpLoad_Buffer[i] = Get_Buffer[i-1];
				UpLoad_Buffer[13] = 0x00;
				UpLoad_Buffer[14] = 0x0A;
				UpLoad_Buffer[15] = 0x00;
				UpLoad_Buffer[16] = 0x01;
				for(i=1;i<17;i++)
					Up_checksum += UpLoad_Buffer[i];
				UpLoad_Buffer[17] = Up_checksum &0xFF;
				UpLoad_Buffer[18] = (Up_checksum>>8) &0xFF;
				Up_checksum = 0;
				for(i=1;i<19;i++)
				{
					if(0xaa==UpLoad_Buffer[i] || 0x55==UpLoad_Buffer[i] )
					{
						CommandFinal[CommandL] = 0x55;
						CommandFinal[CommandL+1] =  (UpLoad_Buffer[i] ^ 0x20);	
						CommandL=CommandL+2;
					}									
					else
				 {
						CommandFinal[CommandL] = UpLoad_Buffer[i];
						CommandL++;
				 }
				}	
				CommandFinal[0] = 0xAA;
				while(GetEPTxStatus(ENDP1)!= EP_TX_NAK);
				CDC_Send_DATA((uint8_t*)CommandFinal,CommandL);
				CommandL = 1;
				StartWork = 1;
			}
				
			if(Get_Buffer[2] == 0x81)//link
			{
				MinThresholdValue = *(uint16_t*) 0x0803f800;
				MaxThresholdValue = *(uint16_t*) 0x0803f802;
				Dac_Voltage_Set = *(uint16_t*) 0x0803f804;
				
				UpLoad_BufferTwo[0]=0xAA;
				
				UpLoad_BufferTwo[1]=0x0E;
				UpLoad_BufferTwo[2]=0x00;
				
				UpLoad_BufferTwo[3]=0x81;
				
				UpLoad_BufferTwo[4]=MaxThresholdValue&0xFF;
				UpLoad_BufferTwo[5]=(MaxThresholdValue>>8)&0xFF;
				UpLoad_BufferTwo[6]=MinThresholdValue&0xFF;
				UpLoad_BufferTwo[7]=(MinThresholdValue>>8)&0xFF;
				
				UpLoad_BufferTwo[8]=MaxLineNum;
				UpLoad_BufferTwo[9]=MaxRowNum;
				
				UpLoad_BufferTwo[10]=RefreshRate&0xFF;
				UpLoad_BufferTwo[11]=(RefreshRate>>8)&0xFF;
				
				UpLoad_BufferTwo[12]=Dac_Voltage_Set;
				
				UpLoad_BufferTwo[13]=0x00;
				UpLoad_BufferTwo[14]=0x0A;
				UpLoad_BufferTwo[15]=0x00;
				UpLoad_BufferTwo[16]=0x01;
				
				for(i=1;i<17;i++)
					sum_ubt += UpLoad_BufferTwo[i];
				UpLoad_BufferTwo[17]=sum_ubt & 0xFF;
				UpLoad_BufferTwo[18]=(sum_ubt>>8)& 0xFF;
				sum_ubt = 0;
				for(i=1;i<19;i++)
				{
					if(0xaa==UpLoad_BufferTwo[i] || 0x55==UpLoad_BufferTwo[i] )
					{
						CommandFinalT[CommandLT] = 0x55;
						CommandFinalT[CommandLT+1] =  (UpLoad_BufferTwo[i] ^ 0x20);	
						CommandL=CommandLT+2;
					}									
					else
					{
						CommandFinalT[CommandLT] = UpLoad_BufferTwo[i];
						CommandLT++;
					}
				}	
				CommandFinalT[0] = 0xAA;
				while(GetEPTxStatus(ENDP1)!= EP_TX_NAK);
				CDC_Send_DATA((uint8_t*)CommandFinalT,CommandLT);
				CommandLT = 1;			
			}		
	}
		if(StartWork == 1)
		{
			StartWork = 0;
			U2_ENABLE;
			S0_L;
			S1_L;
			S2_L;
			S3_L;
			U7_DISABLE;
			U2_DISABLE;
			U1_DISABLE;
			U8_DISABLE;
			Line_Channel = 1;
			U2_ENABLE;
			DAC_SetChannel1Data(DAC1,DAC_Align_12b_R, (drive_voltage_set*Dac_Voltage_Set*0.1)/3.3*4096);
			DAC_SoftwareTriggerCmd(DAC1,DAC_Channel_1,ENABLE);
			TIM_Cmd(TIM3,ENABLE);
		}			
		if(Line_over_flag==1)
		{
			Line_Num++;
			Line_Switch();
			Line_over_flag = 0;
			if(Line_Num == 64)
			{
				TIM_Cmd(TIM3,DISABLE);
				//Value_Final[0][0] = 2000;
				for(i=0;i<64;i++)
					for(j=0;j<64;j++)
				{
					 if(Value_Final[i][j]>0)
						  Value_Final[i][j]=(float)39.49*Value_Final[i][j]/ Dac_Voltage_Set;
						// Value_Final[i][j]=(float)59.24*Value_Final[i][j]/ (drive_voltage_set*Dac_Voltage_Set);
						// Value_Final[i][j]=(float)118.48*MaxThresholdValue/10000*Value_Final[i][j]/ (drive_voltage_set*Dac_Voltage_Set);
						//Value_Final[i][j]=(float)236.96*MaxThresholdValue/10000*Value_Final[i][j]/ (drive_voltage_set*2*Dac_Voltage_Set);
					//Value_Final[i][j]=Value_Final[i][j];
				}
				Line_Num = 0;		
				U2_DISABLE;
				U1_DISABLE;
				U8_DISABLE;
				U7_DISABLE;					
				ProcessData();
				TransData();
				Blink_Led();					
				U2_ENABLE;
        TIM_Cmd(TIM3,ENABLE);					
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
