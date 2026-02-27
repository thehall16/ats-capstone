#ifndef UART_DISPLAY_H
#define UART_DISPLAY_H

#include <stdint.h>

/*
 * uart_printf
 *
 * Formatted UART output using USART2.
 * Uses the global HAL UART handle defined in main.c.
 */
void uart_printf(const char *fmt, ...);

#endif /* UART_DISPLAY_H */