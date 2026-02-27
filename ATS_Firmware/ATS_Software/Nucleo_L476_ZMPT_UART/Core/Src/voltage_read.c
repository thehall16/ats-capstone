#include "voltage_read.h"
#include "main.h"

#include <math.h>

/* Reuse ADC handle created in main.c */
extern ADC_HandleTypeDef hadc1;

/* Calibration Constants */
static const uint16_t ADC_OFFSET = 3080;     /* hard-coded midpoint for ZMPT output */
static const float    LINE_SCALE = 185.5f;   /* module Vrms -> line Vrms scale factor */
static const float    ADC_VREF   = 3.3f;     /* ADC reference voltage */

float voltage_get_module_rms(uint16_t samples)
{
    float sumSq = 0.0f;

    for (uint16_t i = 0; i < samples; i++)
    {
        if (HAL_ADC_Start(&hadc1) == HAL_OK)
        {
            if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
            {
                uint32_t raw = HAL_ADC_GetValue(&hadc1);

                /* Remove DC offset */
                int32_t ac_raw = (int32_t)raw - (int32_t)ADC_OFFSET;

                /* Convert to volts */
                float v = (ac_raw * ADC_VREF) / 4095.0f;

                sumSq += v * v;
            }
            HAL_ADC_Stop(&hadc1);
        }
    }

    return sqrtf(sumSq / (float)samples);
}

float voltage_get_line_vrms(uint16_t samples)
{
    float module_rms = voltage_get_module_rms(samples);
    float line_vrms  = module_rms * LINE_SCALE;

    /* Clamp: below ~20V reading is junk/noise */
    if (line_vrms < 20.0f)
        line_vrms = 0.0f;

    return line_vrms;
}