#include "data_collect.h"
#include "main.h"
#include <stdio.h>
#include <stdlib.h>

// =============================================================================
// Private Defines and Type Definitions
// =============================================================================
#define NUM_ADCS DATA_COLLECT_NUM_ADCS // Declared for convenience
#define BUF_LENGTH_PER_CHANNEL 128

#define ADC1_NUM_ANALOG_CHANNELS 4
#define ADC1_BUF_LENGTH (BUF_LENGTH_PER_CHANNEL * ADC1_NUM_ANALOG_CHANNELS)
#define ADC2_NUM_ANALOG_CHANNELS 5
#define ADC2_BUF_LENGTH (BUF_LENGTH_PER_CHANNEL * ADC2_NUM_ANALOG_CHANNELS)
#define ADC3_NUM_ANALOG_CHANNELS 3
#define ADC3_BUF_LENGTH (BUF_LENGTH_PER_CHANNEL * ADC3_NUM_ANALOG_CHANNELS)
#define ADC4_NUM_ANALOG_CHANNELS 3
#define ADC4_BUF_LENGTH (BUF_LENGTH_PER_CHANNEL * ADC4_NUM_ANALOG_CHANNELS)

#define MAX_NUM_ANALOG_CHANNELS 5

#define ADC1_RESULTS_STORED 0x1
#define ADC2_RESULTS_STORED 0x2
#define ADC3_RESULTS_STORED 0x4
#define ADC4_RESULTS_STORED 0x8
#define ALL_RESULTS_STORED  0xf

typedef enum {
    FIRST_HALF = 0,
    SECOND_HALF = 1
} TransferHalf_t;

typedef enum {
    ADC1_ID = (uint32_t) ADC1,
    ADC2_ID = (uint32_t) ADC2,
    ADC3_ID = (uint32_t) ADC3,
    ADC4_ID = (uint32_t) ADC4
} AdcId_t;

// =============================================================================
// Global Variables
// =============================================================================

// Circular buffers for DMA transfers from ADCs
static volatile uint16_t adc1_buf[ADC1_BUF_LENGTH] = {0};
static volatile uint16_t adc2_buf[ADC2_BUF_LENGTH] = {0};
static volatile uint16_t adc3_buf[ADC3_BUF_LENGTH] = {0};
static volatile uint16_t adc4_buf[ADC4_BUF_LENGTH] = {0};

static volatile uint32_t storedResults[DATA_COLLECT_TOTAL_NUM_ANALOG_CHANNELS] = {0};
static volatile uint32_t storedFlags = 0;

// =============================================================================
// Private Function Prototypes
// =============================================================================
void ProcessReadings(TransferHalf_t half, volatile uint16_t *adc_buf,
                     uint32_t result[MAX_NUM_ANALOG_CHANNELS], uint32_t num_analog_channels);

// =============================================================================
// Public Function Definitions
// =============================================================================

void DataCollect_PrepareHardware(ADC_HandleTypeDef *hadc1, ADC_HandleTypeDef *hadc2,
                                 ADC_HandleTypeDef *hadc3, ADC_HandleTypeDef *hadc4)
{
    if (HAL_ADC_Start_DMA(hadc1, (uint32_t *) adc1_buf, ADC1_BUF_LENGTH) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_ADC_Start_DMA(hadc2, (uint32_t *) adc2_buf, ADC2_BUF_LENGTH) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_ADC_Start_DMA(hadc3, (uint32_t *) adc3_buf, ADC3_BUF_LENGTH) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_ADC_Start_DMA(hadc4, (uint32_t *) adc4_buf, ADC4_BUF_LENGTH) != HAL_OK)
    {
        Error_Handler();
    }
}

void DataCollect_Start(TIM_HandleTypeDef *triggerTimerHandle)
{
    HAL_TIM_Base_Start(triggerTimerHandle);
}

void DataCollect_Stop(TIM_HandleTypeDef *triggerTimerHandle)
{
    HAL_TIM_Base_Stop(triggerTimerHandle);
}

int DataCollect_Poll(void)
{
    return storedFlags == ALL_RESULTS_STORED;
}

void DataCollect_Get(uint32_t buffer[DATA_COLLECT_TOTAL_NUM_ANALOG_CHANNELS])
{
    for(int i = 0; i < DATA_COLLECT_TOTAL_NUM_ANALOG_CHANNELS; i++)
    {
        buffer[i] = storedResults[i];
    }

    storedFlags = 0;
}

// =============================================================================
// Interrupt Callback Definitions
// =============================================================================

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    uint32_t result[MAX_NUM_ANALOG_CHANNELS];
    // Average 1st half of the buffer
    switch ((uint32_t) hadc->Instance)
    {
        case ADC1_ID:
            ProcessReadings(FIRST_HALF, adc1_buf, result, ADC1_NUM_ANALOG_CHANNELS);
            break;
        case ADC2_ID:
            ProcessReadings(FIRST_HALF, adc2_buf, result, ADC2_NUM_ANALOG_CHANNELS);
            break;
        case ADC3_ID:
            ProcessReadings(FIRST_HALF, adc3_buf, result, ADC3_NUM_ANALOG_CHANNELS);
            break;
        case ADC4_ID:
            ProcessReadings(FIRST_HALF, adc4_buf, result, ADC4_NUM_ANALOG_CHANNELS);
            break;
        default:
            Error_Handler();
            break;
    }

    // convertTemp(result[0]);
}

// Conversion complete DMA interrupt callback
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    uint32_t result[MAX_NUM_ANALOG_CHANNELS];
    // Average 2nd half of the buffer
    switch ((uint32_t) hadc->Instance)
    {
        case ADC1_ID:
            ProcessReadings(SECOND_HALF, adc1_buf, result, ADC1_NUM_ANALOG_CHANNELS);
            break;
        case ADC2_ID:
            ProcessReadings(SECOND_HALF, adc2_buf, result, ADC2_NUM_ANALOG_CHANNELS);
            break;
        case ADC3_ID:
            ProcessReadings(SECOND_HALF, adc3_buf, result, ADC3_NUM_ANALOG_CHANNELS);
            break;
        case ADC4_ID:
            ProcessReadings(SECOND_HALF, adc4_buf, result, ADC4_NUM_ANALOG_CHANNELS);
            break;
        default:
            Error_Handler();
            break;
    }

    // convertTemp(result[0]);
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
    printf("\nDMA TRANSFER ERROR\n");
    Error_Handler();
}

// =============================================================================
// Private Function Definitions
// =============================================================================

void ProcessReadings(TransferHalf_t half, volatile uint16_t *adc_buf,
                     uint32_t result[MAX_NUM_ANALOG_CHANNELS], uint32_t num_analog_channels)
{
    uint32_t sum[MAX_NUM_ANALOG_CHANNELS] = {0};
    uint32_t sample_num;
    uint32_t sample_start_index;
    uint32_t limit;

    if (half == FIRST_HALF)
    {
        sample_num = 0;
        limit = BUF_LENGTH_PER_CHANNEL / 2;
    }
    else
    {
        sample_num = BUF_LENGTH_PER_CHANNEL / 2;
        limit = BUF_LENGTH_PER_CHANNEL;
    }

    // Sum the samples
    for (; sample_num < limit; sample_num++)
    {
        sample_start_index = sample_num * num_analog_channels;
        for (int channel = 0; channel < num_analog_channels; channel++)
        {
            sum[channel] += adc_buf[sample_start_index + channel];
        }
    }

    for (int channel = 0; channel < num_analog_channels; channel++)
    {
        // If average should be taken instead
        // result[channel] = ((float)sum[channel]) / (BUF_LENGTH_PER_CHANNEL / 2);

        result[channel] = sum[channel];
    }
}

void StoreResults(uint32_t results[], AdcId_t adcId)
{
    // Mapping of inputs to thermistor indices is defined here
    switch (adcId)
    {
        case ADC1_ID:
            storedResults[0] = results[0];
            storedResults[1] = results[1];
            storedResults[2] = results[2];
            storedResults[3] = results[3];
            storedFlags |= ADC1_RESULTS_STORED;
            break;
        case ADC2_ID:
            storedResults[4] = results[0];
            storedResults[5] = results[1];
            storedResults[6] = results[2];
            storedResults[7] = results[3];
            storedResults[8] = results[4];
            storedFlags |= ADC2_RESULTS_STORED;
            break;
        case ADC3_ID:
            storedResults[9] = results[0];
            storedResults[10] = results[1];
            storedResults[11] = results[2];
            storedFlags |= ADC3_RESULTS_STORED;
            break;
        case ADC4_ID:
            storedResults[12] = results[0];
            storedResults[13] = results[1];
            storedResults[14] = results[2];
            storedFlags |= ADC4_RESULTS_STORED;
            break;
        default:
            Error_Handler();
            break;
    }
}
