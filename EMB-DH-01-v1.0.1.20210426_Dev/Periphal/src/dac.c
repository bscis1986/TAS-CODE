#include "dac.h"
#include "stm32f30x_dac.h"
#include "protocal.h"
#include "parameter_common.h"

void drivesignal_set(float dac_voltage)
{
	DAC_SetChannel1Data(DAC1,DAC_Align_12b_R, (dac_voltage * frame_data.parameter_data_receive.parameter_data.dac_voltage_set * 0.1f) / P_ADC_REF * 4096);
	DAC_SoftwareTriggerCmd(DAC1,DAC_Channel_1,ENABLE);
}