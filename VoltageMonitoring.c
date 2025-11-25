// Voltage Monitoring 
// MCU: STM32C031K6T6
// Sensor: ZMPT101B (with adjustable potentiometer)
// Author: Spencer Grow

#include <stdio.h>
#include <math.h>
#include "stm32c0xx_hal.h"
#include "generator.h"
#include "Display.h"

// Voltage thresholds
float low_voltage  = 86.0f;     // Minimum acceptable AC voltage (RMS)
float high_voltage = 128.0f;    // Maximum acceptable AC voltage (RMS)

// GPIO Port
#define RELAY_PORT GPIOA

// Relay Pins
#define MAIN_SSR_PIN       GPIO_PIN_1
#define BACKUP_SSR_PIN     GPIO_PIN_2
#define SPDT_PIN           GPIO_PIN_6

// LED indicator pins (GEN steps)
#define CHOKE_LED_PIN      GPIO_PIN_3
#define START_LED_PIN      GPIO_PIN_4
#define RUN_LED_PIN        GPIO_PIN_5

// Delay before switching SPDT after SSR changes
#define SAFETY_DELAY_MS 300

// Function prototypes
ADC_HandleTypeDef hadc1;
float Read_Voltage_RMS(void);

typedef enum {
    SOURCE_MAIN,
    SOURCE_BACKUP
} PowerSource;

PowerSource current_source = SOURCE_MAIN;  // Assume main AC on startup

void SystemClock_Config(void);  // Provided by CubeIDE autocode
void MX_GPIO_Init(void);
void MX_ADC1_Init(void);


//MAIN PROGRAM
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_ADC1_Init();
    display_init();              // Initialize 7-segment Display

    while (1)
    {
        // Read RMS voltage
        float voltage = Read_Voltage_RMS();

        // Update 7-segment display
        display_voltage(voltage);

        // NORMAL GRID OK 
        if (voltage >= low_voltage && voltage <= high_voltage)
        {
            if (current_source != SOURCE_MAIN)
            {
                // Turn off backup SSR
                HAL_GPIO_WritePin(RELAY_PORT, BACKUP_SSR_PIN, GPIO_PIN_RESET);

                // Stop generator sequence
                generator_stop();

                HAL_Delay(SAFETY_DELAY_MS);

                // Switch SPDT to MAIN
                HAL_GPIO_WritePin(RELAY_PORT, SPDT_PIN, GPIO_PIN_RESET);

                // Enable main SSR
                HAL_GPIO_WritePin(RELAY_PORT, MAIN_SSR_PIN, GPIO_PIN_SET);

                current_source = SOURCE_MAIN;
            }
        }

        // GRID OUT OF RANGE
        else
        {
            if (current_source != SOURCE_BACKUP)
            {
                // Turn off main SSR
                HAL_GPIO_WritePin(RELAY_PORT, MAIN_SSR_PIN, GPIO_PIN_RESET);

                // Start generator sequence
                generator_startup_sequence();

                HAL_Delay(SAFETY_DELAY_MS);

                // Switch SPDT to BACKUP
                HAL_GPIO_WritePin(RELAY_PORT, SPDT_PIN, GPIO_PIN_SET);

                // Turn on backup SSR
                HAL_GPIO_WritePin(RELAY_PORT, BACKUP_SSR_PIN, GPIO_PIN_SET);

                current_source = SOURCE_BACKUP;
            }
        }

        HAL_Delay(200); // main loop delay
    }
}


// RMS Voltage Measurement
float Read_Voltage_RMS(void)
{
    const uint16_t samples = 200;
    const float ADC_MID = 2048.0f;  // 12-bit midpoint
    float sum_sq = 0.0f;

    for (uint16_t i = 0; i < samples; i++)
    {
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
        uint16_t adc_val = HAL_ADC_GetValue(&hadc1);
        HAL_ADC_Stop(&hadc1);

        float centered = (float)adc_val - ADC_MID;  // remove DC bias
        sum_sq += centered * centered;
    }

    float adc_rms = sqrtf(sum_sq / samples);

    // Convert ADC RMS → Volts RMS (TEMP FACTOR)
    float sensor_voltage = (adc_rms * 3.3f) / 4096.0f;

    // ZMPT101B calibration multiplier — adjusted via board potentiometer later
    float voltage_rms = sensor_voltage * 100.0f;

    return voltage_rms;
}


// GPIO Initialization
void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // ADC pin (PA0)
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Output pins: SSRs + SPDT + LED indicators
    GPIO_InitStruct.Pin =
          MAIN_SSR_PIN
        | BACKUP_SSR_PIN
        | SPDT_PIN
        | CHOKE_LED_PIN
        | START_LED_PIN
        | RUN_LED_PIN;

    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}


// ADC Initialization
void MX_ADC1_Init(void)
{
    __HAL_RCC_ADC1_CLK_ENABLE();

    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.ScanConvMode          = ADC_SCAN_DISABLE;
    hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    hadc1.Init.ContinuousConvMode    = DISABLE;
    hadc1.Init.NbrOfConversion       = 1;
    hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;

    HAL_ADC_Init(&hadc1);

    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel      = ADC_CHANNEL_0;   // PA0
    sConfig.Rank         = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;

    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
}
