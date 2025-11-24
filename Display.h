// display.h - Header for HT16K33 4-digit 7-segment display
#ifndef DISPLAY_H
#define DISPLAY_H

#include "stm32c0xx_hal.h"

// Initialize the 7-segment display (call once at startup)
int init_display(void);

// Shutdown / clear the display
int shutdown_display(void);

// Write RMS voltage to display (whole numbers only)
int write_display(float voltage);

#endif // DISPLAY_H