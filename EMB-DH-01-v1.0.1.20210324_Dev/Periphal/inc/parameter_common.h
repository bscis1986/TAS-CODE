#ifndef __PARAMETER_COMMON_H
#define __PARAMETER_COMMON_H
#include "protocal.h"
#if debug
    #define P_TIMER               (*((uint16_t*)(0x2000ff00)))
		#define P_AMP_FACTOR          ((*((uint16_t*)(0x2000ff02))))
		#define P_DRIVESIGNAL				  ((*((uint16_t*)(0x2000ff04)))*0.001f)
		#define P_ADC_REF             ((*((uint16_t*)(0x2000ff06)))*0.01f)
#else
		#define P_TIMER               60
		#define P_AMP_FACTOR          1
		#define P_DRIVESIGNAL				  1.5f
		#define P_ADC_REF             3.3f	
#endif
#define SAMPLE_RATE            10
#define FLITER_FACTOR          0.32f
#endif
