#ifndef __PARAMETER_COMMON_H
#define __PARAMETER_COMMON_H

#define P_TIMER               (*((uint16_t*)(0x2000ff00)))
#define P_AMP_FACTOR          ((*((uint16_t*)(0x2000ff02)))*0.01f)
#define P_DRIVESIGNAL				  ((*((uint16_t*)(0x2000ff04)))*0.001f)
#define P_FILTER_FACTOR       ((*((uint16_t*)(0x2000ff06)))*0.001f)
#define P_FILTER_AMP          ((*((uint16_t*)(0x2000ff08)))*0.0001f)
#define P_ADC_REF             (*((uint16_t*)(0x2000ff0A)))
#define P_LOCATION_NUM        (*((uint16_t*)(0x2000ff0C)))
#define P_PARAMETER_FACTOR    (*((uint16_t*)(0x2000ff0E)))
#endif
