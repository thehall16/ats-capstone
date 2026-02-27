#include "uart_display.h"
#include "main.h"

#include <stdio.h>
#include <stdarg.h>


extern UART_HandleTypeDef huart2;

void uart_printf(const char *fmt, ...)
{
    char buffer[512]; //adjust if uart isn't printing full message

    va_list args;
    va_start(args, fmt);

    int len = vsnprintf(buffer, sizeof(buffer), fmt, args);

    va_end(args);

    if (len <= 0)
        return;

    /* Clamp length to buffer size */
    if (len > (int)sizeof(buffer))
        len = (int)sizeof(buffer);

    HAL_UART_Transmit(&huart2,
                      (uint8_t *)buffer,
                      (uint16_t)len,
                      HAL_MAX_DELAY);
}