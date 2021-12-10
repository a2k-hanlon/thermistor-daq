#ifndef __DATA_COLLECT_H
#define __DATA_COLLECT_H

#include "stm32f3xx_hal.h"

#define DATA_COLLECT_NUM_ADCS 4
#define DATA_COLLECT_TOTAL_NUM_ANALOG_CHANNELS 15

void DataCollect_PrepareHardware(ADC_HandleTypeDef *hadc1, ADC_HandleTypeDef *hadc2,
                                 ADC_HandleTypeDef *hadc3, ADC_HandleTypeDef *hadc4);
void DataCollect_Start(TIM_HandleTypeDef *triggerTimerHandle);
void DataCollect_Stop(TIM_HandleTypeDef *triggerTimerHandle);
int  DataCollect_Poll(void);
void DataCollect_Get(uint32_t buffer[DATA_COLLECT_TOTAL_NUM_ANALOG_CHANNELS]);

#endif // __DATA_COLLECT_H
