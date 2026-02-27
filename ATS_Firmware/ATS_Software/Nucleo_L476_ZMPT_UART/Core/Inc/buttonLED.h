#ifndef BUTTONLED_H
#define BUTTONLED_H

/* Call once after MX_GPIO_Init + MX_USART2_UART_Init */
void ButtonLED_Init(void);

/* Call repeatedly in while(1) */
void ButtonLED_Task(void);

#endif /* BUTTONLED_H */