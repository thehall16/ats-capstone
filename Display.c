//Display Functionality

//Display current Main voltage
// display.c - STM32 HT16K33 4-digit 7-segment display

#include "stm32c0xx_hal.h"
#include "display.h"

// I2C handle (configured in CubeIDE)
extern I2C_HandleTypeDef hi2c1;

#define DISPLAY_ADDR (0x70 << 1)  // 7-bit HT16K33 address shifted for STM32 HAL

// 7-segment patterns for digits 0-9
static const uint8_t digits[] = {
    0x3F, // 0
    0x06, // 1
    0x5B, // 2
    0x4F, // 3
    0x66, // 4
    0x6D, // 5
    0x7D, // 6
    0x07, // 7
    0x7F, // 8
    0x6F  // 9
};

// Segment codes for letters/symbols
#define SEG_E 0x79
#define SEG_r 0x50

// HT16K33 buffer
static uint8_t buffer[16];

//Initialize display
int init_display(void)
{
    HAL_Delay(50); // wait for display power-up

    uint8_t cmd;

    // Turn on oscillator
    cmd = 0x21;
    if(HAL_I2C_Master_Transmit(&hi2c1, DISPLAY_ADDR, &cmd, 1, HAL_MAX_DELAY) != HAL_OK) return -1;

    // Display on, no blink
    cmd = 0x81;
    if(HAL_I2C_Master_Transmit(&hi2c1, DISPLAY_ADDR, &cmd, 1, HAL_MAX_DELAY) != HAL_OK) return -1;

    // Set brightness (max 15)
    cmd = 0xEF;
    if(HAL_I2C_Master_Transmit(&hi2c1, DISPLAY_ADDR, &cmd, 1, HAL_MAX_DELAY) != HAL_OK) return -1;

    // Clear buffer
    for(int i=0; i<16; i++) buffer[i] = 0;
    HAL_I2C_Master_Transmit(&hi2c1, DISPLAY_ADDR, buffer, 16, HAL_MAX_DELAY);

    return 0;
}

//Shutdown display
int shutdown_display(void)
{
    // Clear display buffer
    for(int i=0; i<16; i++) buffer[i] = 0;
    HAL_I2C_Master_Transmit(&hi2c1, DISPLAY_ADDR, buffer, 16, HAL_MAX_DELAY);
    return 0;
}

//Write RMS voltage to display
int write_display(float voltage)
{
    // Clear buffer
    for(int i=0;i<16;i++) buffer[i] = 0;

    // Invalid voltage (>150 V)
    if(voltage > 150.0f)
    {
        buffer[0] = SEG_E;
        buffer[2] = SEG_r;
        buffer[4] = SEG_r;
        return HAL_I2C_Master_Transmit(&hi2c1, DISPLAY_ADDR, buffer, 16, HAL_MAX_DELAY);
    }

    int int_voltage = (int) voltage;  // whole number only

    // Split digits
    int d1 = int_voltage / 100;          // hundreds
    int d2 = (int_voltage / 10) % 10;    // tens
    int d3 = int_voltage % 10;           // ones
    

    // Blank leading zeros
    if(int_voltage < 100) d1 = -1;
    if(int_voltage < 10)  d2 = -1;

    // Assign digits to buffer (HT16K33 format)
    if(d1 >= 0) buffer[0] = digits[d1];
    if(d2 >= 0) buffer[2] = digits[d2];
    buffer[4] = digits[d3];   // ones digit
    

    // Send buffer to display
    return HAL_I2C_Master_Transmit(&hi2c1, DISPLAY_ADDR, buffer, 16, HAL_MAX_DELAY);
}
